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
  state_->time = std::chrono::steady_clock::now();
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
  jsd_slave_config_.product_code         = JSD_EL3318_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  jsd_slave_config_.el3318.element[0] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[1] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[2] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[3] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[4] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[5] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[6] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.element[7] = JSD_EL3318_ELEMENT_TYPE_K;
  jsd_slave_config_.el3318.filter[0] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[1] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[2] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[3] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[4] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[5] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[6] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.filter[7] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3318.presentation[0] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[1] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[2] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[3] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[4] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[5] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[6] = JSD_EL3318_PRESENTATION_SIGNED;
  jsd_slave_config_.el3318.presentation[7] = JSD_EL3318_PRESENTATION_SIGNED;

  return true;
}

bool fastcat::El3318::Read()
{
  jsd_el3318_read((jsd_t*)context_, slave_id_);

  state_->time = std::chrono::steady_clock::now();
  const jsd_el3318_state_t* jsd_state =
      jsd_el3318_get_state((jsd_t*)context_, slave_id_);

  state_->el3318_state.output_eu_ch1   = jsd_state->output_eu[0];
  state_->el3318_state.output_eu_ch2   = jsd_state->output_eu[1];
  state_->el3318_state.output_eu_ch3   = jsd_state->output_eu[2];
  state_->el3318_state.output_eu_ch4   = jsd_state->output_eu[3];
  state_->el3318_state.output_eu_ch5   = jsd_state->output_eu[4];
  state_->el3318_state.output_eu_ch6   = jsd_state->output_eu[5];
  state_->el3318_state.output_eu_ch7   = jsd_state->output_eu[6];
  state_->el3318_state.output_eu_ch8   = jsd_state->output_eu[7];
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
