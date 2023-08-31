// Include related header (for cc files)
#include "fastcat/jsd/jed0200.h"

// Include c then c++ libraries
#include <string.h>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Jed0200::Jed0200()
{
  MSG_DEBUG("Constructed Jed0200");

  state_       = std::make_shared<DeviceState>();
  state_->type = JED0200_STATE;
}

bool fastcat::Jed0200::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}
bool fastcat::Jed0200::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_JED0200_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "initial_cmd", initial_cmd_)) {
    WARNING("JED0200 intial_cmd unspecified, defaulting to 0");
    initial_cmd_ = 0;
  }

  return true;
}

bool fastcat::Jed0200::Read()
{
  jsd_jed0200_read((jsd_t*)context_, slave_id_);

  const jsd_jed0200_state_t* jsd_state =
      jsd_jed0200_get_state((jsd_t*)context_, slave_id_);

  state_->jed0200_state.status        = jsd_state->status;
  state_->jed0200_state.ticks         = jsd_state->ticks;
  state_->jed0200_state.voltage_hv    = jsd_state->voltage_hv;
  state_->jed0200_state.voltage_lv    = jsd_state->voltage_lv;
  state_->jed0200_state.voltage_12v   = jsd_state->voltage_12v;
  state_->jed0200_state.temp_ambient  = jsd_state->temp_ambient;
  state_->jed0200_state.temp_actuator = jsd_state->temp_actuator;
  state_->jed0200_state.humidity      = jsd_state->humidity;
  state_->jed0200_state.pressure      = jsd_state->pressure;
  state_->jed0200_state.brake_current = jsd_state->brake_current;
  state_->jed0200_state.brake_cc_val  = jsd_state->brake_cc_val;
  state_->jed0200_state.cmd           = jsd_state->cmd;

  return true;
}

fastcat::FaultType fastcat::Jed0200::Process()
{
  jsd_jed0200_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::Jed0200::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == JED0200_SET_CMD_VALUE_CMD) {
    jsd_jed0200_set_cmd_value((jsd_t*)context_, slave_id_,
                              cmd.jed0200_set_cmd_value_cmd.cmd);

  } else {
    WARNING("Bad JED0200 Command");
    return false;
  }
  return true;
}
