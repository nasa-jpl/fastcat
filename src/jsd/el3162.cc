// Include related header (for cc files)
#include "fastcat/jsd/el3162.h"

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El3162::El3162()
{
  MSG_DEBUG("Constructed El3162");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL3162_STATE;
}

bool fastcat::El3162::ConfigFromYaml(YAML::Node node, double external_time)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El3162::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL3162_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El3162::Read()
{
  jsd_el3162_read((jsd_t*)context_, slave_id_);

  const jsd_el3162_state_t* jsd_state =
      jsd_el3162_get_state((jsd_t*)context_, slave_id_);

  state_->el3162_state.voltage_ch1   = jsd_state->voltage[0];
  state_->el3162_state.voltage_ch2   = jsd_state->voltage[1];
  state_->el3162_state.adc_value_ch1 = jsd_state->adc_value[0];
  state_->el3162_state.adc_value_ch2 = jsd_state->adc_value[1];

  return true;
}