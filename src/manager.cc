// Include related header (for cc files)
#include "fastcat/manager.h"

// Include c then c++ libraries
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <typeindex>
#include <typeinfo>

// Include external then project includes
#include "fastcat/fastcat_devices/commander.h"
#include "fastcat/fastcat_devices/conditional.h"
#include "fastcat/fastcat_devices/faulter.h"
#include "fastcat/fastcat_devices/filter.h"
#include "fastcat/fastcat_devices/fts.h"
#include "fastcat/fastcat_devices/function.h"
#include "fastcat/fastcat_devices/pid.h"
#include "fastcat/fastcat_devices/saturation.h"
#include "fastcat/fastcat_devices/schmitt_trigger.h"
#include "fastcat/fastcat_devices/signal_generator.h"
#include "fastcat/fastcat_devices/virtual_fts.h"
#include "fastcat/jsd/actuator.h"
#include "fastcat/jsd/actuator_offline.h"
#include "fastcat/jsd/ati_fts.h"
#include "fastcat/jsd/ati_fts_offline.h"
#include "fastcat/jsd/egd.h"
#include "fastcat/jsd/egd_offline.h"
#include "fastcat/jsd/el2124.h"
#include "fastcat/jsd/el2124_offline.h"
#include "fastcat/jsd/el3208.h"
#include "fastcat/jsd/el3208_offline.h"
#include "fastcat/jsd/el3602.h"
#include "fastcat/jsd/el3602_offline.h"
#include "fastcat/jsd/el3104.h"
#include "fastcat/jsd/el3104_offline.h"
#include "fastcat/jsd/el3202.h"
#include "fastcat/jsd/el3202_offline.h"
#include "fastcat/jsd/el3318.h"
#include "fastcat/jsd/el3318_offline.h"
#include "fastcat/jsd/jed.h"
#include "fastcat/jsd/jed_offline.h"
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Manager::Manager()
{
  cmd_queue_ = std::make_shared<std::queue<DeviceCmd>>();
}

fastcat::Manager::~Manager()
{
  for (auto it = jsd_map_.begin(); it != jsd_map_.end(); ++it) {
    if (it->second != NULL) {
      jsd_free(it->second);
    }
  }
  SUCCESS("Freed JSD memory");
}

void fastcat::Manager::Shutdown()
{
  GetActuatorPositions();
  SaveActuatorPosFile();
}

bool fastcat::Manager::ConfigFromYaml(YAML::Node node)
{
  // Configure Fastcat Parameters
  YAML::Node fastcat_node;
  if (!ParseNode(node, "fastcat", fastcat_node)) {
    return false;
  }

  if (!ParseVal(fastcat_node, "target_loop_rate_hz", target_loop_rate_hz_)) {
    return false;
  }

  if (!ParseVal(fastcat_node, "zero_latency_required",
                zero_latency_required_)) {
    return false;
  }

  if (!ParseVal(fastcat_node, "actuator_position_directory",
                actuator_position_directory_)) {
    return false;
  }

  if (!ParseVal(fastcat_node, "actuator_fault_on_missing_pos_file",
                actuator_fault_on_missing_pos_file_)) {
    return false;
  }

  // load pos file startup positions
  if (!LoadActuatorPosFile()) {
    return false;
  }

  // Configure Buses
  YAML::Node buses_node;
  if (!ParseList(node, "buses", buses_node)) {
    return false;
  }

  for (auto bus = buses_node.begin(); bus != buses_node.end(); ++bus) {
    std::string type;
    if (!ParseVal(*bus, "type", type)) {
      return false;
    }

    if (0 == type.compare("jsd_bus")) {
      if (!ConfigJSDBusFromYaml(*bus)) {
        ERROR("Failed to configure JSD bus");
        return false;
      }

    } else if (0 == type.compare("fastcat_bus")) {
      if (!ConfigFastcatBusFromYaml(*bus)) {
        ERROR("Failed to configure Fastcat bus");
        return false;
      }

    } else if (0 == type.compare("offline_bus")) {
      if (!ConfigOfflineBusFromYaml(*bus)) {
        ERROR("Failed to configure Offline bus");
        return false;
      }

    } else {
      ERROR("Could not parse bus type %s", type.c_str());
      return false;
    }
  }

  SUCCESS("Added %lu devices to map", device_map_.size());

  MSG("JSD device list entries: ");
  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    MSG("\t%s", (*it)->GetName().c_str());
  }

  if (!ValidateActuatorPosFile()) {
    ERROR("Failed to validate Actuator startup positions");
    return false;
  }

  MSG("Unsorted Fastcat Devices: ");
  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    MSG("\t%s", (*it)->GetName().c_str());
  }

  std::vector<std::shared_ptr<DeviceBase>> sorted_list;
  std::vector<std::string>                 parents;
  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    if (!SortFastcatDevice(*it, sorted_list, parents)) {
      return false;
    }
  }

  fastcat_device_list_.swap(sorted_list);

  MSG("Sorted Fastcat Devices: ");
  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    MSG("\t%s", (*it)->GetName().c_str());
  }

  MSG("Configuring Signals...");
  if (!ConfigSignals()) {
    ERROR("Could not configure Signals");
    return false;
  }
  SUCCESS("Configured Signals.");

  MSG("Reading initial state of all devices.");
  for (auto it = jsd_map_.begin(); it != jsd_map_.end(); ++it) {
    jsd_read(it->second, 1e9 / target_loop_rate_hz_);
  }

  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    if (!(*it)->Read()) {
      WARNING("Bad Process on %s", (*it)->GetName().c_str());
    }
  }

  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    if (!(*it)->Read()) {
      WARNING("Bad Process on %s", (*it)->GetName().c_str());
    }
  }

  if (!SetActuatorPositions()) {
    return false;
  }

  return true;
}

bool fastcat::Manager::Process()
{
  for (auto it = jsd_map_.begin(); it != jsd_map_.end(); ++it) {
    jsd_read(it->second, 1e9 / target_loop_rate_hz_);
  }

  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    if (!(*it)->Read()) {
      WARNING("Bad Process on %s", (*it)->GetName().c_str());
    }
  }

  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    if (!(*it)->Read()) {
      WARNING("Bad Process on %s", (*it)->GetName().c_str());
    }
    switch ((*it)->Process()) {
      case WARNING:
        WARNING("Process Warning %s", (*it)->GetName().c_str());
        break;
      case ALL_DEVICE_FAULT:
        ERROR("Process Error %s", (*it)->GetName().c_str());
        ExecuteAllDeviceFaults();
        break;
      default:
        break;
    }
  }

  WriteCommands();

  // JSD devices are processed after the call to WriteCommands() because some
  // JSD devices need their internal state to be updated based on the changes
  // made by the queued commands.
  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    switch ((*it)->Process()) {
      case WARNING:
        WARNING("Process Warning %s", (*it)->GetName().c_str());
        break;
      case ALL_DEVICE_FAULT:
        ERROR("Process Error %s", (*it)->GetName().c_str());
        ExecuteAllDeviceFaults();
        break;
      default:
        break;
    }
  }

  for (auto it = jsd_map_.begin(); it != jsd_map_.end(); ++it) {
    jsd_write(it->second);
  }

  return !faulted_;
}

void fastcat::Manager::QueueCommand(DeviceCmd& cmd) { cmd_queue_->push(cmd); }

std::vector<fastcat::DeviceState> fastcat::Manager::GetDeviceStates()
{
  states_.clear();
  states_.resize(device_map_.size());
  auto device_pair = device_map_.begin();

  for (size_t i = 0; i < device_map_.size(); ++i) {
    states_[i] = *device_pair->second->GetState();
    ++device_pair;
  }
  return states_;
}

std::vector<std::shared_ptr<const fastcat::DeviceState>>
fastcat::Manager::GetDeviceStatePointers()
{
  std::vector<std::shared_ptr<const DeviceState>> state_ptrs;
  state_ptrs.resize(device_map_.size());

  auto device_pair = device_map_.begin();

  for (size_t i = 0; i < device_map_.size(); ++i) {
    state_ptrs[i] = device_pair->second->GetState();
    ++device_pair;
  }
  return state_ptrs;
}

double fastcat::Manager::GetTargetLoopRate() { return target_loop_rate_hz_; }
bool   fastcat::Manager::IsFaulted() { return faulted_; }

bool fastcat::Manager::RecoverBus(std::string ifname)
{
  MSG("RecoverBus: %s", ifname.c_str());
  auto find_pair = jsd_map_.find(ifname);

  if (find_pair == jsd_map_.end()) {
    WARNING("Bad Bus name %s", ifname.c_str());
    return false;
  }
  MSG("Device found!");

  jsd_set_manual_recovery(find_pair->second);
  return true;
}

bool fastcat::Manager::ConfigJSDBusFromYaml(YAML::Node node)
{
  std::string ifname;
  if (!ParseVal(node, "ifname", ifname)) {
    return false;
  }

  bool enable_ar;
  if (!ParseVal(node, "enable_autorecovery", enable_ar)) {
    return false;
  }
  YAML::Node devices_node;
  if (!ParseList(node, "devices", devices_node)) {
    return false;
  }

  jsd_t* jsd = jsd_alloc();

  JSDPair pair(ifname, jsd);
  jsd_map_.insert(pair);

  std::shared_ptr<DeviceBase> device;
  uint16_t                    slave_id = 0;

  for (auto device_node = devices_node.begin();
       device_node != devices_node.end(); ++device_node) {
    slave_id++;
    std::string device_class;
    if (!ParseVal(*device_node, "device_class", device_class)) {
      return false;
    }

    if (0 == device_class.compare("IGNORE")) {
      continue;
    }

    // device specific
    if (0 == device_class.compare("Egd")) {
      device = std::make_shared<Egd>();

    } else if (0 == device_class.compare("El3208")) {
      device = std::make_shared<El3208>();

    } else if (0 == device_class.compare("El3602")) {
      device = std::make_shared<El3602>();

    } else if (0 == device_class.compare("El2124")) {
      device = std::make_shared<El2124>();

    } else if (0 == device_class.compare("El3104")) {
      device = std::make_shared<El3104>();

    } else if (0 == device_class.compare("El3202")) {
      device = std::make_shared<El3202>();

    } else if (0 == device_class.compare("El3318")) {
      device = std::make_shared<El3318>();

    } else if (0 == device_class.compare("Actuator")) {
      device = std::make_shared<Actuator>();

    } else if (0 == device_class.compare("Jed")) {
      device = std::make_shared<Jed>();

    } else if (0 == device_class.compare("AtiFts")) {
      device = std::make_shared<AtiFts>();

    } else {
      ERROR("Unknown device_class: %s", device_class.c_str());
      return false;
    }

    device->SetLoopPeriod(1.0 / target_loop_rate_hz_);
    device->SetContext((void*)jsd);
    device->SetSlaveId(slave_id);
    if (!device->ConfigFromYaml(*device_node)) {
      ERROR("Failed to configure after the first %lu devices",
            device_map_.size());
      return false;
    }

    if (!CheckDeviceNameIsUnique(device->GetName())) {
      return false;
    }

    DevicePair pair(device->GetName(), device);
    device_map_.insert(pair);
    jsd_device_list_.push_back(device);
  }

  return jsd_init(jsd, ifname.c_str(), enable_ar);
}

bool fastcat::Manager::ConfigFastcatBusFromYaml(YAML::Node node)
{
  std::string ifname;
  if (!ParseVal(node, "ifname", ifname)) {
    return false;
  }

  YAML::Node devices_node;
  if (!ParseList(node, "devices", devices_node)) {
    return false;
  }

  std::shared_ptr<DeviceBase> device;
  for (auto device_node = devices_node.begin();
       device_node != devices_node.end(); ++device_node) {
    std::string device_class;
    if (!ParseVal(*device_node, "device_class", device_class)) {
      return false;
    }

    if (0 == device_class.compare("IGNORE")) {
      continue;
    }

    // device specific
    if (0 == device_class.compare("Commander")) {
      device = std::make_shared<Commander>();

    } else if (0 == device_class.compare("SignalGenerator")) {
      device = std::make_shared<SignalGenerator>();

    } else if (0 == device_class.compare("Function")) {
      device = std::make_shared<Function>();

    } else if (0 == device_class.compare("Conditional")) {
      device = std::make_shared<Conditional>();

    } else if (0 == device_class.compare("Pid")) {
      device = std::make_shared<Pid>();

    } else if (0 == device_class.compare("Saturation")) {
      device = std::make_shared<Saturation>();

    } else if (0 == device_class.compare("SchmittTrigger")) {
      device = std::make_shared<SchmittTrigger>();

    } else if (0 == device_class.compare("Filter")) {
      device = std::make_shared<Filter>();

    } else if (0 == device_class.compare("Fts")) {
      device = std::make_shared<Fts>();

    } else if (0 == device_class.compare("VirtualFts")) {
      device = std::make_shared<VirtualFts>();

    } else if (0 == device_class.compare("Faulter")) {
      device = std::make_shared<Faulter>();

    } else {
      ERROR("Unknown device_class: %s", device_class.c_str());
      return false;
    }

    device->SetLoopPeriod(1.0 / target_loop_rate_hz_);
    if (!device->ConfigFromYaml(*device_node)) {
      ERROR("Failed to configure after the first %lu devices",
            device_map_.size());
      return false;
    }

    if (!CheckDeviceNameIsUnique(device->GetName())) {
      return false;
    }

    DevicePair pair(device->GetName(), device);
    device_map_.insert(pair);
    fastcat_device_list_.push_back(device);
  }

  return true;
}

bool fastcat::Manager::ConfigOfflineBusFromYaml(YAML::Node node)
{
  // Here include any relevant bus level offline EGD parameters that may be
  // neccesary, e.g.
  // uint8_t plant_model;
  // if (!ParseVal(node, "plant_model", plant_model)) {
  //  return false;
  //}
  std::string ifname;
  if (!ParseVal(node, "ifname", ifname)) {
    return false;
  }

  YAML::Node devices_node;
  if (!ParseList(node, "devices", devices_node)) {
    return false;
  }

  std::shared_ptr<DeviceBase> device;
  uint16_t                    slave_id = 0;

  for (auto device_node = devices_node.begin();
       device_node != devices_node.end(); ++device_node) {
    slave_id++;
    std::string device_class;
    if (!ParseVal(*device_node, "device_class", device_class)) {
      return false;
    }

    if (0 == device_class.compare("IGNORE")) {
      continue;
    }

    // device specific
    if (0 == device_class.compare("Egd")) {
      device = std::make_shared<EgdOffline>();

    } else if (0 == device_class.compare("El2124")) {
      device = std::make_shared<El2124Offline>();

    } else if (0 == device_class.compare("El3208")) {
      device = std::make_shared<El3208Offline>();

    } else if (0 == device_class.compare("El3602")) {
      device = std::make_shared<El3602Offline>();

    } else if (0 == device_class.compare("El3104")) {
      device = std::make_shared<El3104Offline>();

    } else if (0 == device_class.compare("El3202")) {
      device = std::make_shared<El3202Offline>();

    } else if (0 == device_class.compare("El3318")) {
      device = std::make_shared<El3318Offline>();

    } else if (0 == device_class.compare("Actuator")) {
      device = std::make_shared<ActuatorOffline>();

    } else if (0 == device_class.compare("Jed")) {
      device = std::make_shared<JedOffline>();

    } else if (0 == device_class.compare("AtiFts")) {
      device = std::make_shared<AtiFtsOffline>();

    } else {
      ERROR("Unknown device_class: %s", device_class.c_str());
      return false;
    }

    device->SetLoopPeriod(1.0 / target_loop_rate_hz_);
    device->SetSlaveId(slave_id);
    if (!device->ConfigFromYaml(*device_node)) {
      ERROR("Failed to configure after the first %lu devices",
            device_map_.size());
      return false;
    }

    if (!CheckDeviceNameIsUnique(device->GetName())) {
      return false;
    }

    DevicePair pair(device->GetName(), device);
    device_map_.insert(pair);
    jsd_device_list_.push_back(device);
  }

  return true;
}

bool fastcat::Manager::WriteCommands()
{
  if (faulted_) {
    // Clear device command queue
    while (!cmd_queue_->empty()) {
      cmd_queue_->pop();
    }
    return true;
  }

  while (!cmd_queue_->empty()) {
    DeviceCmd cmd = cmd_queue_->front();
    cmd_queue_->pop();

    auto find_pair = device_map_.find(cmd.name);

    if (find_pair == device_map_.end()) {
      WARNING("Bad command device name");
      return false;
    }

    if (!find_pair->second->Write(cmd)) {
      WARNING("Bad Write for %s", find_pair->first.c_str());
      ExecuteAllDeviceFaults();
      return false;
    }
  }
  return true;
}

bool fastcat::Manager::ConfigSignals()
{
  for (auto it = device_map_.begin(); it != device_map_.end(); ++it) {
    auto device = it->second;

    device->RegisterCmdQueue(cmd_queue_);

    for (auto signal = device->signals_.begin();
         signal != device->signals_.end(); ++signal) {
      // we cannot use the [] operator as it will create a new entry
      // we cannot use the at() method as it will raise an exception
      // so find() it is

      auto find_pair = device_map_.find(signal->observed_device_name);

      if (find_pair == device_map_.end()) {
        if (signal->observed_device_name.compare("FIXED_VALUE") != 0) {
          ERROR("Did not find an Observed device name for %s",
                device->GetName().c_str());
        }
        continue;
      }

      auto observed_state = find_pair->second->GetState();

      if (!ConfigSignalByteIndexing(observed_state.get(), *signal)) {
        ERROR("Could not configure the Signal Byte Indexing");
        return false;
      }
    }
  }
  return true;
}

bool fastcat::Manager::SortFastcatDevice(
    std::shared_ptr<DeviceBase>               device,
    std::vector<std::shared_ptr<DeviceBase>>& sorted_devices,
    std::vector<std::string>                  parents)
{
  std::string dev_name = device->GetName();
  MSG_DEBUG("Checking %s", dev_name.c_str());

  if (std::find(sorted_devices.begin(), sorted_devices.end(), device) ==
      sorted_devices.end()) {
    MSG_DEBUG("Device not found in sorted list");
    parents.push_back(dev_name);
    size_t ii = 0;
    for (ii = 0; ii < device->signals_.size(); ii++) {
      std::string child = device->signals_[ii].observed_device_name;
      if (child.compare("FIXED_VALUE") != 0) {
        MSG_DEBUG("Checking child %s", child.c_str());
        if (std::find(parents.begin(), parents.end(), child) != parents.end()) {
          if (zero_latency_required_) {
            ERROR(
                "Cyclical signal loop detected. Devices %s and %s are mutually "
                "reliant. Zero latency cannot be enforced.",
                dev_name.c_str(), child.c_str());
            return false;
          } else {
            WARNING(
                "Cyclical signal loop detected. Devices %s and %s are mutually "
                "reliant. Zero latency cannot be enforced.",
                dev_name.c_str(), child.c_str());
          }
        } else {
          auto device_ptr = device_map_.find(child);

          // Check if device exists
          if (device_ptr == device_map_.end()) {
            ERROR("Device %s signal observed_device_name: %s does not exist!",
                  dev_name.c_str(), child.c_str());
            return false;
          }

          if (!SortFastcatDevice(device_ptr->second, sorted_devices, parents)) {
            return false;
          }
        }
      }
    }

    if (std::find(jsd_device_list_.begin(), jsd_device_list_.end(), device) ==
        jsd_device_list_.end()) {
      MSG_DEBUG("Adding device %s to sorted list.", dev_name.c_str());

      if (ii > 0) {
        sorted_devices.push_back(device);
      } else {
        sorted_devices.insert(sorted_devices.begin(), device);
      }
    }
  }

  return true;
}

bool fastcat::Manager::ExecuteDeviceFault(std::string device_name)
{
  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    if (device_name.compare((*it)->GetName()) == 0) {
      (*it)->Fault();
      return true;
    }
  }

  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    if (device_name.compare((*it)->GetName()) == 0) {
      (*it)->Fault();
      return true;
    }
  }

  return false;
}

bool fastcat::Manager::ExecuteDeviceReset(std::string device_name)
{
  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    if (device_name.compare((*it)->GetName()) == 0) {
      (*it)->Reset();
      return true;
    }
  }

  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    if (device_name.compare((*it)->GetName()) == 0) {
      (*it)->Reset();
      return true;
    }
  }

  return false;
}

void fastcat::Manager::ExecuteAllDeviceFaults()
{
  if (faulted_) {
    return;
  }

  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    (*it)->Fault();
  }

  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    (*it)->Fault();
  }

  faulted_ = true;
}

void fastcat::Manager::ExecuteAllDeviceResets()
{
  for (auto it = jsd_device_list_.begin(); it != jsd_device_list_.end(); ++it) {
    (*it)->Reset();
  }

  for (auto it = fastcat_device_list_.begin(); it != fastcat_device_list_.end();
       ++it) {
    (*it)->Reset();
  }
  faulted_ = false;
}

bool fastcat::Manager::LoadActuatorPosFile()
{
  if (!actuator_fault_on_missing_pos_file_) {
    WARNING("YAML parameter \'actuator_fault_on_missing_pos_file\' is FALSE");
    WARNING("\tThis setting is intended for demo and testing and should");
    WARNING("\tnot be used for deployments running actuators in production.");
  }

  if (actuator_position_directory_.empty()) {
    ERROR("actuator_position_directory is empty, check the YAML parameter");
    return false;
  }
  std::string pos_file =
      actuator_position_directory_ + "/fastcat_saved_positions.yaml";

  struct stat st;
  if (0 != stat(pos_file.c_str(), &st)) {
    if (actuator_fault_on_missing_pos_file_) {
      ERROR("Failed to open pos file: %s", strerror(errno));
      return false;
    } else {
      WARNING("Continuing without pos file: %s", strerror(errno));
      return true;
    }
  }

  MSG_DEBUG("Opening Pos File: %s", pos_file.c_str());

  YAML::Node node = YAML::LoadFile(pos_file);
  if (!node) {
    ERROR("Could not parse pos file YAML: %s", pos_file.c_str());
    return false;
  }

  YAML::Node actuators_node;
  if (!ParseNode(node, "actuators", actuators_node)) {
    return false;
  }

  for (auto act_node = actuators_node.begin(); act_node != actuators_node.end();
       ++act_node) {
    std::string name;
    if (!ParseVal(*act_node, "actuator_name", name)) {
      return false;
    }

    ActuatorPosData act_pos_data;
    if (!ParseVal(*act_node, "position", act_pos_data.position)) {
      return false;
    }

    actuator_pos_map_[name] = act_pos_data;
    MSG_DEBUG(" Act: %s startup_position: %lf", name.c_str(),
              act_pos_data.position);
  }

  return true;
}

bool fastcat::Manager::ValidateActuatorPosFile()
{
  // Now need to check that the loaded position file matches the
  //   actuators created by the topology.yaml
  //
  // if return value is true, there will be an entry in the
  //   actuator_pos_map_  for each actuator in the bus topology

  // Warn if unused entry is found in position file
  for (auto saved_pos_entry = actuator_pos_map_.begin();
       saved_pos_entry != actuator_pos_map_.end(); ++saved_pos_entry) {
    auto find_pair = device_map_.find(saved_pos_entry->first);

    if (find_pair == device_map_.end()) {
      WARNING("Unused saved position entry found for: %s",
              saved_pos_entry->first.c_str());
    }
  }

  // Err if actuator is created by topology but no pos file entry exists
  std::shared_ptr<DeviceState> dev_state;
  std::string                  dev_name;
  for (auto device = jsd_device_list_.begin(); device != jsd_device_list_.end();
       ++device) {
    dev_state = (*device)->GetState();
    dev_name  = (*device)->GetName();

    if (dev_state->type != ACTUATOR_STATE) {
      continue;
    }
    auto find_pos_data = actuator_pos_map_.find(dev_name);

    if (find_pos_data == actuator_pos_map_.end()) {
      if (!actuator_fault_on_missing_pos_file_) {
        WARNING(
            "Missing Startup position for %s, setting starting position to "
            "zero",
            dev_name.c_str());

        ActuatorPosData apd         = {0};
        actuator_pos_map_[dev_name] = apd;

      } else {
        ERROR("Missing startup position for %s", dev_name.c_str());
        return false;
      }
    }
  }
  return true;
}

bool fastcat::Manager::SetActuatorPositions()
{
  std::shared_ptr<DeviceState> dev_state;
  std::string                  dev_name;

  for (auto device = jsd_device_list_.begin(); device != jsd_device_list_.end();
       ++device) {
    dev_state = (*device)->GetState();
    dev_name  = (*device)->GetName();

    if (dev_state->type != ACTUATOR_STATE) {
      continue;
    }
    auto find_pos_data = actuator_pos_map_.find(dev_name);

    MSG("Setting actuator: %s to saved pos: %lf", dev_name.c_str(),
        find_pos_data->second.position);

    if (!(*device)->SetOutputPosition(find_pos_data->second.position)) {
      ERROR("Failure on SetOutputPosition for device: %s", dev_name.c_str());
      return false;
    }
  }

  return true;
}

void fastcat::Manager::GetActuatorPositions()
{
  std::shared_ptr<DeviceState> dev_state;
  std::string                  dev_name;
  for (auto device = jsd_device_list_.begin(); device != jsd_device_list_.end();
       ++device) {
    dev_state = (*device)->GetState();
    dev_name  = (*device)->GetName();

    if (dev_state->type != ACTUATOR_STATE) {
      continue;
    }

    ActuatorPosData apd         = {0};
    apd.position                = dev_state->actuator_state.actual_position;
    actuator_pos_map_[dev_name] = apd;

    MSG("Actuator: %s position is %lf", dev_name.c_str(), apd.position);
  }
}

void fastcat::Manager::SaveActuatorPosFile()
{
  std::string prev_pos_file =
      actuator_position_directory_ + "/fastcat_saved_positions_prev.yaml";
  std::string pos_file =
      actuator_position_directory_ + "/fastcat_saved_positions.yaml";

  MSG("Renaming %s -> %s", pos_file.c_str(), prev_pos_file.c_str());

  if (0 != rename(pos_file.c_str(), prev_pos_file.c_str())) {
    WARNING("Could not move: %s, file may not exist", pos_file.c_str());
  }

  YAML::Node file_node;
  for (auto pos_pair = actuator_pos_map_.begin();
       pos_pair != actuator_pos_map_.end(); ++pos_pair) {
    YAML::Node act_node;
    act_node["actuator_name"] = pos_pair->first;
    act_node["position"]      = pos_pair->second.position;
    file_node["actuators"].push_back(act_node);
  }

  umask(000);
  std::ofstream file(pos_file, std::ios::out);
  file << file_node;
  file.close();
  MSG("Successfully wrote Pos File: %s", pos_file.c_str());
}

bool fastcat::Manager::CheckDeviceNameIsUnique(std::string name)
{
  if (unique_device_map_.end() != unique_device_map_.find(name)) {
    ERROR("Device %s is not unique, check your Fastcat config!", name.c_str());
    return false;
  }
  unique_device_map_[name] = true;
  return true;
}
