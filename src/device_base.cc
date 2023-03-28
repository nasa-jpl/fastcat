// Include related header (for cc files)
#include "fastcat/device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"


fastcat::DeviceBase::DeviceBase(DeviceType device_type){
  device_type_ = device_type;
  state_ = std::make_shared<DeviceState>();
  state_->type = device_type;
}

// Non-pure virtual methods with default implementation
fastcat::FaultType fastcat::DeviceBase::Process() { return NO_FAULT; }

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


// Non-virtual methods

void fastcat::DeviceBase::RegisterCmdQueue(
    std::shared_ptr<std::queue<DeviceCmd>> cmd_queue)
{
  cmd_queue_ = cmd_queue;
}

std::string fastcat::DeviceBase::GetName() { return config_.name; }

fastcat::DeviceType fastcat::DeviceBase::GetDeviceType() { return device_type_; }

std::shared_ptr<fastcat::DeviceState> fastcat::DeviceBase::GetState()
{
  return state_;
}

fastcat::DeviceConfig fastcat::DeviceBase::GetConfig() { return config_; }


void fastcat::DeviceBase::SetConfig(DeviceConfig config){
  config_ = config;
  name_ = config.name;
  // TODO print config using print macros
}	

void fastcat::DeviceBase::SetTime(double time) { state_->time = time; }

void fastcat::DeviceBase::SetLoopPeriod(double loop_period)
{
  loop_period_ = loop_period;
}

