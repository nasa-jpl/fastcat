// Include related header (for cc files)
#include "fastcat/jsd/el1008.h"

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El1008::El1008()
{
  MSG_DEBUG("Constructed El1008");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL1008_STATE;
}

bool fastcat::El1008::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El1008::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.driver_type          = JSD_DRIVER_TYPE_EL1008;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El1008::Read()
{
  jsd_el1008_read((jsd_t*)context_, slave_id_);

  const jsd_el1008_state_t* jsd_state =
      jsd_el1008_get_state((jsd_t*)context_, slave_id_);

  state_->el1008_state.level_ch1   = jsd_state->values[0];
  state_->el1008_state.level_ch2   = jsd_state->values[1];
  state_->el1008_state.level_ch3   = jsd_state->values[2];
  state_->el1008_state.level_ch4   = jsd_state->values[3];
  state_->el1008_state.level_ch5   = jsd_state->values[4];
  state_->el1008_state.level_ch6   = jsd_state->values[5];
  state_->el1008_state.level_ch7   = jsd_state->values[6];
  state_->el1008_state.level_ch8   = jsd_state->values[7];

  return true;
}