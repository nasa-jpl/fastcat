// Include related header (for cc files)
#include "fastcat/jsd/jed0101_offline.h"

// Include c then c++ libraries
#include <string.h>

// Include external then project includes
#include "jsd/jsd_print.h"

bool fastcat::Jed0101Offline::ConfigFromYaml(const YAML::Node& node,
                                             double /*external_time*/)
{
  bool retval               = ConfigFromYamlCommon(node);
  state_->jed0101_state.cmd = initial_cmd_;
  return retval;
}

bool fastcat::Jed0101Offline::Read() { return true; }

fastcat::FaultType fastcat::Jed0101Offline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::Jed0101Offline::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == JED0101_SET_CMD_VALUE_CMD) {
    state_->jed0101_state.cmd = cmd.jed0101_set_cmd_value_cmd.cmd;
  } else {
    WARNING("that command type is not supported yet!");
    return false;
  }
  return true;
}
