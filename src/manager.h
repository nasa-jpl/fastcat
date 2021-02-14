#ifndef FASTCAT_MANAGER_H_
#define FASTCAT_MANAGER_H_

// Include related header (for cc files)

// Include c then c++ libraries
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

// Include external then project includes
#include <yaml-cpp/yaml.h>

#include "fastcat/device_base.h"
#include "jsd/jsd_print.h"

namespace fastcat
{
typedef std::pair<std::string, std::shared_ptr<DeviceBase>> DevicePair;
typedef std::pair<std::string, jsd_t*>                      JSDPair;

class Manager
{
 public:
  Manager();
  ~Manager();

  void Shutdown();
  bool ConfigFromYaml(YAML::Node node);
  bool Process();
  void QueueCommand(DeviceCmd& cmd);

  std::vector<DeviceState> GetDeviceStates();
  std::vector<std::shared_ptr<const DeviceState>> GetDeviceStatePointers();
  double                   GetTargetLoopRate();
  bool                     IsFaulted();

  bool RecoverBus(std::string ifname);
  bool ExecuteDeviceReset(std::string device_name);
  bool ExecuteDeviceFault(std::string device_name);
  void ExecuteAllDeviceResets();
  void ExecuteAllDeviceFaults();

 private:
  bool ConfigJSDBusFromYaml(YAML::Node node);
  bool ConfigFastcatBusFromYaml(YAML::Node node);
  bool ConfigOfflineBusFromYaml(YAML::Node node);
  bool WriteCommands();
  bool ConfigSignals();
  bool SortFastcatDevice(
      std::shared_ptr<DeviceBase>               device,
      std::vector<std::shared_ptr<DeviceBase>>& sorted_devices,
      std::vector<std::string>                  parents);

  bool LoadActuatorPosFile();
  bool ValidateActuatorPosFile();
  bool SetActuatorPositions();
  void GetActuatorPositions();
  void SaveActuatorPosFile();
  bool CheckDeviceNameIsUnique(std::string name);

  double                        target_loop_rate_hz_;
  bool                          zero_latency_required_              = true;
  bool                          faulted_                            = false;
  bool                          actuator_fault_on_missing_pos_file_ = true;
  std::string                   actuator_position_directory_;
  std::map<std::string, jsd_t*> jsd_map_;

  std::map<std::string, std::shared_ptr<DeviceBase>> device_map_;
  std::vector<std::shared_ptr<DeviceBase>>           fastcat_device_list_;
  std::vector<std::shared_ptr<DeviceBase>>           jsd_device_list_;
  std::shared_ptr<std::queue<DeviceCmd>>             cmd_queue_;
  std::vector<DeviceState>                           states_;
  std::map<std::string, ActuatorPosData>             actuator_pos_map_;
  std::unordered_map<std::string, bool>              unique_device_map_;
};
}  // namespace fastcat

#endif
