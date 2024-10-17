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
#include "fastcat/thread_safe_queue.h"

namespace fastcat
{
class DeviceBase
{
 public:
  virtual ~DeviceBase();
  // Pure virtual methods
  virtual bool ConfigFromYaml(const YAML::Node& node) = 0;
  virtual bool Read()                                 = 0;

  // Non-pure virtual methods with default implementation
  virtual FaultType Process();
  virtual bool      Write(DeviceCmd& cmd);
  virtual void      Fault();
  virtual void      Reset();
  virtual void SetInitializationTime(double time_sec, double monotonic_time_sec)
  {
    initialization_time_sec_           = time_sec;
    monotonic_initialization_time_sec_ = monotonic_time_sec;
  }

  // non-virtual methods
  void RegisterCmdQueue(std::shared_ptr<ThreadSafeQueue<DeviceCmd>> cmd_queue);
  std::string                  GetName();
  std::shared_ptr<DeviceState> GetState();

  void SetTime(double time, double monotonic_time);
  void SetLoopPeriod(double loop_period);

  std::vector<Signal> signals_;

 protected:
  std::string name_;  ///< unique device name

  double last_monotonic_time_               = 0.0;
  double loop_period_                       = 0.0;  ///< only some devices need
  double initialization_time_sec_           = -1;   ///< only some devices need
  double monotonic_initialization_time_sec_ = -1;   ///< only some devices need

  /// device-level fault, manager also has fault status flag
  bool device_fault_active_ = false;

  std::shared_ptr<DeviceState> state_;  ///< Fastcat state data

  /// for intra-device commands
  std::shared_ptr<ThreadSafeQueue<DeviceCmd>> cmd_queue_;
};

}  // namespace fastcat

#endif
