// Include related header (for cc files)
#include "fastcat/jsd/el3318.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El3318::El3318()
{
  MSG_DEBUG("Constructed El3318");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL3318_STATE;
}

bool fastcat::El3318::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El3318::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.driver_type          = JSD_DRIVER_TYPE_EL3318;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "element", element_string_)) {
    return false;
  }

  if (!ElementFromString(element_string_, element_)) {
    return false;
  }

  int i;
  for (i = 0; i < JSD_EL3318_NUM_CHANNELS; i++) {
    jsd_slave_config_.el3318.element[i]      = element_;
    jsd_slave_config_.el3318.filter[i]       = JSD_BECKHOFF_FILTER_50HZ;
    jsd_slave_config_.el3318.presentation[i] = JSD_EL3318_PRESENTATION_SIGNED;
  }

  return true;
}

bool fastcat::El3318::Read()
{
  jsd_el3318_read((jsd_t*)context_, slave_id_);

  const jsd_el3318_state_t* jsd_state =
      jsd_el3318_get_state((jsd_t*)context_, slave_id_);

  state_->el3318_state.output_eu_ch1 = jsd_state->output_eu[0];
  state_->el3318_state.output_eu_ch2 = jsd_state->output_eu[1];
  state_->el3318_state.output_eu_ch3 = jsd_state->output_eu[2];
  state_->el3318_state.output_eu_ch4 = jsd_state->output_eu[3];
  state_->el3318_state.output_eu_ch5 = jsd_state->output_eu[4];
  state_->el3318_state.output_eu_ch6 = jsd_state->output_eu[5];
  state_->el3318_state.output_eu_ch7 = jsd_state->output_eu[6];
  state_->el3318_state.output_eu_ch8 = jsd_state->output_eu[7];
  state_->el3318_state.adc_value_ch1 = jsd_state->adc_value[0];
  state_->el3318_state.adc_value_ch2 = jsd_state->adc_value[1];
  state_->el3318_state.adc_value_ch3 = jsd_state->adc_value[2];
  state_->el3318_state.adc_value_ch4 = jsd_state->adc_value[3];
  state_->el3318_state.adc_value_ch5 = jsd_state->adc_value[4];
  state_->el3318_state.adc_value_ch6 = jsd_state->adc_value[5];
  state_->el3318_state.adc_value_ch7 = jsd_state->adc_value[6];
  state_->el3318_state.adc_value_ch8 = jsd_state->adc_value[7];

  return true;
}

bool fastcat::El3318::ElementFromString(std::string           element_string,
                                        jsd_el3318_element_t& element)
{
  MSG("Converting element to string.");
  if (element_string.compare("TYPE_K") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_K;
  } else if (element_string.compare("TYPE_J") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_J;
  } else if (element_string.compare("TYPE_L") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_L;
  } else if (element_string.compare("TYPE_E") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_E;
  } else if (element_string.compare("TYPE_T") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_T;
  } else if (element_string.compare("TYPE_N") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_N;
  } else if (element_string.compare("TYPE_U") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_U;
  } else if (element_string.compare("TYPE_B") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_B;
  } else if (element_string.compare("TYPE_R") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_R;
  } else if (element_string.compare("TYPE_S") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_S;
  } else if (element_string.compare("TYPE_C") == 0) {
    element = JSD_EL3318_ELEMENT_TYPE_C;
  } else {
    ERROR("El3318 element %s is invalid", element_string.c_str());
    return false;
  }

  return true;
}
