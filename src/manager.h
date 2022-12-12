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
#include "fastcat/jsd/jsd_device_base.h"

#include "jsd/jsd.h"

namespace fastcat
{
typedef std::pair<std::string, std::shared_ptr<DeviceBase>> DevicePair;
typedef std::pair<std::string, jsd_t*>                      JSDPair;

/** @brief Fastcat::Manager is the main application interface to manage all fastcat devices 
 */
class Manager
{
 public:
  Manager();
  ~Manager();

  /** @brief Shutdown the bus and joins all threads. 
   *
   * Also Writes Actuator positions to file if applicable. 
   * @return void
   */
  void Shutdown();

  /** @brief Method that accepts a fastcat topology yaml and intializes bus
   *
   *  @return true on successful initialization. If false, application should quit.
   */
  bool ConfigFromYaml(YAML::Node node);

  /** @brief Updates synchronous PDO and background async SDO requests.
   *
   *  Process() proceeds by
   *  1. Triggers PDO Read on EtherCAT bus
   *  2. Reads all JSD devices into the manager
   *  3. Reads all Fastcat devices (including observed device state propagation)
   *  4. Calls DeviceBase::Process() on Fastcat Devices, checking for faults and SDO requests
   *  5. Writes Device Commands (Includes User commands and Fastcat Device Commands)
   *  6. Calls Process() on JSD devices, checking for faults
   *  7. Triggers PDO Write on EtherCAT bus
   *
   *   Note: For best performance, the application should call the Manager::Process() function 
   *     at the same frequency as the input YAML field `target_loop_rate_hz`. This parameter is
   *     needed by certain devices for profiling and filtering.
   *
   *   @return Return true if bus is not faulted, otherwise a bus fault is active.
   */
  bool Process();

  /** @brief Interface to command devices on the bus
   *  
   *  Commands each have a name field and that name is used to dispatch the command to 
   *  the right device. If the provided name is not found on the loaded bus topology, a 
   *  warning message is printed to stdout but no faults are triggered. 
   *
   *  @return void
   */
  void QueueCommand(DeviceCmd& cmd);

  /** @brief Returns list of device states
   *  
   *  @return device states
   */
  std::vector<DeviceState>                        GetDeviceStates();

  /** @brief Returns list of device state pointers
   *  
   *  Provided for potential optimization. GetDeviceStates() generally performs better.
   *  @return device states
   */
  std::vector<std::shared_ptr<const DeviceState>> GetDeviceStatePointers();

  /** @brief Public getter to the YAML `target_loop_rate_hz` parameter
   *  @return loop rate in hz
   */
  double                                          GetTargetLoopRate();

  /** @brief Public getter to the YAML actuators names
   *  @return  
   */  
  void GetActuatorsName(std::vector<std::string>>& names);

  /** @brief Public getter to the YAML actuators parameters
   *  @return vectors 
   */
  void GetActuatorsParams(std::vector<std::vector<double>>& params);

  /** @brief Public getter retrieve fault status
   *  @return true if bus is faulted
   */
  bool                                            IsFaulted();

  /** @brief Attempts to recover a faulty JSD bus by name
   * 
   *  A bus fault example is spurious Working Counter (WKC) error, or perhaps an intended 
   *  power cycle of a configured EtherCAT slave.
   *
   *  @param ifname the name of the JSD bus configured in the input topology YAML
   *  @return true if bus ifname exists, does not indicate the error is fixed.
   */
  bool RecoverBus(std::string ifname);


  /** @brief Triggers a single device to reset
   * 
   *  @param device_name the name of the device to reset, calls its Reset() method
   *  @return true if the device exists, false if device_name is invalid.
   */
  bool ExecuteDeviceReset(std::string device_name);


  /** @brief Triggers a single device to fault
   * 
   *  @param device_name the name of the device to fault, calls its Fault() method
   *  @return true if the device exists, false if device_name is invalid.
   */
  bool ExecuteDeviceFault(std::string device_name);

  /** @brief Triggers all devices to Reset
   * 
   *   This function loops over all devices in the topology and calls their DeviceBase::Reset()
   *   Method. 
   *
   *   @return void
   */
  void ExecuteAllDeviceResets();

  /** @brief Triggers all devices to Fault
   * 
   *   This function loops over all devices in the topology and calls their DeviceBase::Fault()
   *   Method. An example usage is a soft-stop GUI button that can be used to arrest motion.
   *
   *   @return void
   */
  void ExecuteAllDeviceFaults();

  /** @brief checks if the SdoResponse Queue is empty
   *  @return if queue is empty
   */
  bool IsSdoResponseQueueEmpty();

  /** @brief get the result of a background SDO operation 
   *
   *  If the SDO Response queue contains any responses, this function pops the 
   *  oldest value and returns it to the application.
   *
   *  @return true if the return reference 'res' is valid
   */
  bool PopSdoResponseQueue(SdoResponse& res);


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
  std::vector<std::shared_ptr<JsdDeviceBase>>        jsd_device_list_;
  std::shared_ptr<std::queue<DeviceCmd>>             cmd_queue_;
  std::vector<DeviceState>                           states_;
  std::map<std::string, ActuatorPosData>             actuator_pos_map_;
  std::unordered_map<std::string, bool>              unique_device_map_;
  std::shared_ptr<std::queue<SdoResponse>>           sdo_response_queue_;  
};
}  // namespace fastcat

#endif
