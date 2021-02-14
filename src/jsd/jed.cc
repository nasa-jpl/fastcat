// Include related header (for cc files)
#include "fastcat/jsd/jed.h"

// Include c then c++ libraries
#include <string.h>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Jed::Jed()
{
  MSG_DEBUG("Constructed Jed");

  state_       = std::make_shared<DeviceState>();
  state_->type = JED_STATE;
  state_->time = std::chrono::steady_clock::now();
}

bool fastcat::Jed::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}
bool fastcat::Jed::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_JED_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "initial_cmd", initial_cmd_)) {
    WARNING("JED intial_cmd unspecified, defaulting to 0");
    initial_cmd_ = 0;
  }

  return true;
}

bool fastcat::Jed::Read()
{
  jsd_jed_read((jsd_t*)context_, slave_id_);

  state_->time = std::chrono::steady_clock::now();
  const jsd_jed_state_t* jsd_state =
      jsd_jed_get_state((jsd_t*)context_, slave_id_);

  state_->jed_state.status = jsd_state->status;
  state_->jed_state.w_raw  = jsd_state->w_raw;
  state_->jed_state.x_raw  = jsd_state->x_raw;
  state_->jed_state.y_raw  = jsd_state->y_raw;
  state_->jed_state.z_raw  = jsd_state->z_raw;
  state_->jed_state.w      = jsd_state->w;
  state_->jed_state.x      = jsd_state->x;
  state_->jed_state.y      = jsd_state->y;
  state_->jed_state.z      = jsd_state->z;
  state_->jed_state.cmd    = jsd_state->cmd;

  return true;
}

fastcat::FaultType fastcat::Jed::Process()
{
  jsd_jed_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::Jed::Write(DeviceCmd& cmd)
{
  if (cmd.type == JED_SET_CMD_VALUE_CMD) {
    jsd_jed_set_cmd_value((jsd_t*)context_, slave_id_,
                          cmd.jed_set_cmd_value_cmd.cmd);

  } else {
    WARNING("Bad JED Command");
    return false;
  }
  return true;
}
