// Include related header (for cc files)
#include "fastcat/jsd/el3202.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El3202::El3202()
{
  MSG_DEBUG("Constructed El3202");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL3202_STATE;
}

bool fastcat::El3202::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El3202::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL3202_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "element_ch1", element_ch1_string_)) {
    return false;
  }

  if (!ParseVal(node, "element_ch2", element_ch2_string_)) {
    return false;
  }

  if (!ElementFromString(element_ch1_string_, element_ch1_)) {
    return false;
  }

  if (!ElementFromString(element_ch2_string_, element_ch2_)) {
    return false;
  }

  //TODO: Should all config put in fcat_config.yaml??
  jsd_slave_config_.el3202.element[0] = element_ch1_;
  jsd_slave_config_.el3202.element[1] = element_ch2_;

  jsd_slave_config_.el3202.filter[0] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3202.filter[1] = JSD_BECKHOFF_FILTER_50HZ;
  jsd_slave_config_.el3202.connection[0] = JSD_EL3202_CONNECTION_4WIRE;
  jsd_slave_config_.el3202.connection[1] = JSD_EL3202_CONNECTION_4WIRE;
  jsd_slave_config_.el3202.presentation[0] = JSD_EL3202_PRESENTATION_SIGNED;
  jsd_slave_config_.el3202.presentation[1] = JSD_EL3202_PRESENTATION_SIGNED;
  return true;
}

bool fastcat::El3202::Read()
{
  jsd_el3202_read((jsd_t*)context_, slave_id_);

  const jsd_el3202_state_t* jsd_state =
      jsd_el3202_get_state((jsd_t*)context_, slave_id_);

  state_->el3202_state.output_eu_ch1   = jsd_state->output_eu[0];
  state_->el3202_state.output_eu_ch2   = jsd_state->output_eu[1];
  state_->el3202_state.adc_value_ch1 = jsd_state->adc_value[0];
  state_->el3202_state.adc_value_ch2 = jsd_state->adc_value[1];

  return true;
}

bool fastcat::El3202::ElementFromString(std::string element_string,
                                      jsd_el3202_element_t& element)
{
  MSG("Converting element to string.");
  if (element_string.compare("PT100") == 0) {
    element = JSD_EL3202_ELEMENT_PT100;
  } else if (element_string.compare("NI100") == 0) {
    element = JSD_EL3202_ELEMENT_NI100;
  } else if (element_string.compare("PT1000") == 0) {
    element = JSD_EL3202_ELEMENT_PT1000;
  } else if (element_string.compare("PT500") == 0) {
    element = JSD_EL3202_ELEMENT_PT500;
  } else if (element_string.compare("PT200") == 0) {
    element = JSD_EL3202_ELEMENT_PT200;
  } else if (element_string.compare("NI1000") == 0) {
    element = JSD_EL3202_ELEMENT_NI1000;
  } else if (element_string.compare("NI1000_TK1500") == 0) {
    element = JSD_EL3202_ELEMENT_NI1000_TK1500;
  } else if (element_string.compare("NI120") == 0) {
    element = JSD_EL3202_ELEMENT_NI120;
  } else if (element_string.compare("OHMS4096") == 0) {
    element = JSD_EL3202_ELEMENT_OHMS4096;
  } else if (element_string.compare("OHMS1024") == 0) {
    element = JSD_EL3202_ELEMENT_OHMS1024;
  } else if (element_string.compare("KT100") == 0) {
    element = JSD_EL3202_ELEMENT_KT100_ET_AL;
  } else {
    ERROR("El3202 element %s is invalid", element_string.c_str());
    return false;
  }

  return true;
}
