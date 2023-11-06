// Include related header (for cc files)
#include "fastcat/device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"

fastcat::DeviceBase::~DeviceBase() {}

void fastcat::DeviceBase::RegisterCmdQueue(
    std::shared_ptr<fastcat::ThreadSafeQueue<DeviceCmd>> cmd_queue)
{
  cmd_queue_ = cmd_queue;
}

std::string fastcat::DeviceBase::GetName() { return name_; }
std::shared_ptr<fastcat::DeviceState> fastcat::DeviceBase::GetState()
{
  return state_;
}

void fastcat::DeviceBase::SetLoopPeriod(double loop_period)
{
  loop_period_ = loop_period;
}

void fastcat::DeviceBase::SetTime(double time, double monotonic_time)
{
  state_->time           = time;
  state_->monotonic_time = monotonic_time;
}

bool fastcat::DeviceBase::Write(fastcat::DeviceCmd& /* cmd */)
{
  ERROR("Commands are not supported by device %s", name_.c_str());
  return false;
}

void fastcat::DeviceBase::Fault()
{
  WARNING("Faulting device %s", name_.c_str());
  device_fault_active_ = true;
}

void fastcat::DeviceBase::Reset()
{
  MSG("Resetting device %s", name_.c_str());
  device_fault_active_ = false;
}

fastcat::FaultType fastcat::DeviceBase::Process() { return NO_FAULT; }
