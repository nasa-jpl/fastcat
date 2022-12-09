// Include related header (for cc files)
#include "fastcat/device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"

void fastcat::DeviceBase::RegisterCmdQueue(
    std::shared_ptr<std::queue<DeviceCmd>> cmd_queue)
{
  cmd_queue_ = cmd_queue;
}

std::string fastcat::DeviceBase::GetName() { return name_; }
std::shared_ptr<fastcat::DeviceState> fastcat::DeviceBase::GetState()
{
  return state_;
}

void fastcat::DeviceBase::SetSlaveId(uint16_t slave_id)
{
  MSG_DEBUG("Setting slave id to %u", slave_id);
  slave_id_ = slave_id;
}

uint16_t fastcat::DeviceBase::GetSlaveId() { return slave_id_; }
void     fastcat::DeviceBase::SetContext(void* context) { context_ = context; }
void     fastcat::DeviceBase::SetLoopPeriod(double loop_period)
{
  loop_period_ = loop_period;
}

void fastcat::DeviceBase::SetTime(double time){
  state_->time = time;
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

bool fastcat::DeviceBase::SetOutputPosition(double /* position */)
{
  ERROR("SetOutputPosition not defined for Device: %s", name_.c_str());
  return false;
}

double fastcat::DeviceBase::GetActPosMax() {
  ERROR("GetActPosMax not defined for Device: %s", name_.c_str());
  return 0.0;
}

double fastcat::DeviceBase::GetActPosMin() {
  ERROR("GetActPosMin not defined for Device: %s", name_.c_str());
  return 0.0;
}

double fastcat::DeviceBase::GetActVelMax() {
  ERROR("GetActVelMax not defined for Device: %s", name_.c_str());
  return 0.0;
}

double fastcat::DeviceBase::GetActAccMax() {
  ERROR("GetActAccMax not defined for Device: %s", name_.c_str());
  return 0.0;
}

double fastcat::DeviceBase::GetActCurPeak() {
  ERROR("GetActCurPeak not defined for Device: %s", name_.c_str());
  return 0.0;
}

double fastcat::DeviceBase::GetActCurCont() {
  ERROR("GetActCurCont not defined for Device: %s", name_.c_str());
  return 0.0;
}


