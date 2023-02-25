#ifndef FASTCAT_DEVICE_BASE_H_
#define FASTCAT_DEVICE_BASE_H_

// Include related header (for cc files)

// Include c then c++ libraries
#include <memory>
#include <queue>
#include <string>

// Include external then project includes
#include <yaml-cpp/yaml.h>

#include "fastcat/types.h"

namespace fastcat
{
class DeviceBase
{
 public:
  virtual ~DeviceBase();
  // Pure virtual methods
  virtual bool ConfigFromYaml(YAML::Node node) = 0;
  virtual bool Read()                          = 0;

  // Non-pure virtual methods with default implementation
  virtual FaultType Process();
  virtual bool      Write(DeviceCmd& cmd);
  virtual void      Fault();
  virtual void      Reset();

  // non-virtual methods
  void RegisterCmdQueue(std::shared_ptr<std::queue<DeviceCmd>> cmd_queue);
  std::string                  GetName();
  std::shared_ptr<DeviceState> GetState();

  void SetTime(double time);
  void SetLoopPeriod(double loop_period);

  std::vector<Signal> signals_;

 protected:
  std::string name_;    ///< unique device name
  double loop_period_;  ///< only some devices need

  /// device-level fault, manager also has fault status flag
  bool device_fault_active_ = false;                                      

  std::shared_ptr<DeviceState> state_;  ///< Fastcat state data

  /// for intra-device commands
  std::shared_ptr<std::queue<DeviceCmd>> cmd_queue_;  
};

}  // namespace fastcat

#endif
