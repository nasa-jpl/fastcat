// Include related header (for cc files)
#include "fastcat/jsd/el5042.h"

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El5042::El5042()
{
  MSG_DEBUG("Constructed El5042");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL5042_STATE;
}

bool fastcat::El5042::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El5042::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.driver_type          = JSD_DRIVER_TYPE_EL5042;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El5042::Read()
{
  jsd_el5042_read((jsd_t*)context_, slave_id_);

  const jsd_el5042_state_t* jsd_state =
      jsd_el5042_get_state((jsd_t*)context_, slave_id_);

  state_->el5042_state.position_ch1 = jsd_state->position[0];
  state_->el5042_state.warning_ch1  = jsd_state->warning[0];
  state_->el5042_state.error_ch1    = jsd_state->error[0];
  state_->el5042_state.ready_ch1    = jsd_state->ready[0];
  state_->el5042_state.position_ch2 = jsd_state->position[1];
  state_->el5042_state.warning_ch2  = jsd_state->warning[1];
  state_->el5042_state.error_ch2    = jsd_state->error[1];
  state_->el5042_state.ready_ch2    = jsd_state->ready[1];

  return true;
}
