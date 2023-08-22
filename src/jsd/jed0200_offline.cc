// Include related header (for cc files)
#include "fastcat/jsd/jed0200_offline.h"

// Include c then c++ libraries
#include <string.h>

// Include external then project includes
#include "jsd/jsd_print.h"

bool fastcat::Jed0200Offline::ConfigFromYaml(YAML::Node node,
                                             double /*external_time*/)
{
  bool retval               = ConfigFromYamlCommon(node);
  state_->jed0200_state.cmd = initial_cmd_;
  return retval;
}

bool fastcat::Jed0200Offline::Read() { return true; }

fastcat::FaultType fastcat::Jed0200Offline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::Jed0200Offline::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == JED0200_SET_CMD_VALUE_CMD) {
    state_->jed0200_state.cmd = cmd.jed0200_set_cmd_value_cmd.cmd;
  } else {
    WARNING("that command type is not supported yet!");
    return false;
  }
  return true;
}
