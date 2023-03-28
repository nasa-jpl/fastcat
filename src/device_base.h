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
 private:
  DeviceBase(); // Don't want subclasses to use this ctor

 public:
  DeviceBase(DeviceType device_type); // preferred ctor

  // Pure virtual methods
  virtual bool Initialize() = 0;
  virtual bool Read()       = 0;

  // Non-pure virtual methods with default implementation
  virtual FaultType Process();
  virtual bool      Write(DeviceCmd& cmd);
  virtual void      Fault();
  virtual void      Reset();

  // non-virtual methods
  void RegisterCmdQueue(std::shared_ptr<std::queue<DeviceCmd>> cmd_queue);

  std::string                  GetName();
  DeviceType                   GetDeviceType();
  std::shared_ptr<DeviceState> GetState();
  DeviceConfig                 GetConfig();

  void SetConfig(DeviceConfig config);
  void SetTime(double time);
  void SetLoopPeriod(double loop_period);


 protected:
  double      loop_period_;  ///< only some devices need

  /// device-level fault, manager also has fault status flag
  bool device_fault_active_ = false;

  std::shared_ptr<DeviceState> state_;  ///< Fastcat state data
  DeviceConfig config_; ///< Fastcat configuration
  DeviceType device_type_; 
			
  /// for intra-device commands
  std::shared_ptr<std::queue<DeviceCmd>> cmd_queue_;
};

}  // namespace fastcat

#endif
