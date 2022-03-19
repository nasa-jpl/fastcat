// Include related header (for cc files)
#include "fastcat/jsd/jed0101.h"

// Include c then c++ libraries
#include <string.h>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Jed0101::Jed0101()
{
  MSG_DEBUG("Constructed Jed0101");

  state_       = std::make_shared<DeviceState>();
  state_->type = JED0101_STATE;
}

bool fastcat::Jed0101::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}
bool fastcat::Jed0101::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_JED0101_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "initial_cmd", initial_cmd_)) {
    WARNING("JED0101 intial_cmd unspecified, defaulting to 0");
    initial_cmd_ = 0;
  }

  return true;
}

bool fastcat::Jed0101::Read()
{
  jsd_jed0101_read((jsd_t*)context_, slave_id_);

  const jsd_jed0101_state_t* jsd_state =
      jsd_jed0101_get_state((jsd_t*)context_, slave_id_);

  state_->jed0101_state.status = jsd_state->status;
  state_->jed0101_state.w_raw  = jsd_state->w_raw;
  state_->jed0101_state.x_raw  = jsd_state->x_raw;
  state_->jed0101_state.y_raw  = jsd_state->y_raw;
  state_->jed0101_state.z_raw  = jsd_state->z_raw;
  state_->jed0101_state.w      = jsd_state->w;
  state_->jed0101_state.x      = jsd_state->x;
  state_->jed0101_state.y      = jsd_state->y;
  state_->jed0101_state.z      = jsd_state->z;
  state_->jed0101_state.cmd    = jsd_state->cmd;

  return true;
}

fastcat::FaultType fastcat::Jed0101::Process()
{
  jsd_jed0101_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::Jed0101::Write(DeviceCmd& cmd)
{
  if (cmd.type == JED0101_SET_CMD_VALUE_CMD) {
    jsd_jed0101_set_cmd_value((jsd_t*)context_, slave_id_,
                          cmd.jed0101_set_cmd_value_cmd.cmd);

  } else {
    WARNING("Bad JED0101 Command");
    return false;
  }
  return true;
}
