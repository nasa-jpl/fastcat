// Include related header (for cc files)
#include "fastcat/jsd/jed_offline.h"

// Include c then c++ libraries
#include <string.h>

// Include external then project includes
#include "jsd/jsd_print.h"

bool fastcat::JedOffline::ConfigFromYaml(YAML::Node node)
{
  bool retval           = ConfigFromYamlCommon(node);
  state_->jed_state.cmd = initial_cmd_;
  return retval;
}

bool fastcat::JedOffline::Read()
{
  return true;
}

fastcat::FaultType fastcat::JedOffline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::JedOffline::Write(DeviceCmd& cmd)
{
  if (cmd.type == JED_SET_CMD_VALUE_CMD) {
    state_->jed_state.cmd = cmd.jed_set_cmd_value_cmd.cmd;
  } else {
    WARNING("that command type is not supported yet!");
    return false;
  }
  return true;
}
