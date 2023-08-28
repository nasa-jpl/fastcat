// Include related header (for cc files)
#include "fastcat/jsd/el3602.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El3602::El3602()
{
  MSG_DEBUG("Constructed El3602");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL3602_STATE;
}

bool fastcat::El3602::ConfigFromYaml(const YAML::Node& node, double /*external_time*/)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El3602::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL3602_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "range_ch1", range_ch1_string_)) {
    return false;
  }

  if (!ParseVal(node, "range_ch2", range_ch2_string_)) {
    return false;
  }

  if (!RangeFromString(range_ch1_string_, range_ch1_)) {
    return false;
  }

  if (!RangeFromString(range_ch2_string_, range_ch2_)) {
    return false;
  }

  jsd_slave_config_.el3602.range[0]  = range_ch1_;
  jsd_slave_config_.el3602.range[1]  = range_ch2_;
  jsd_slave_config_.el3602.filter[0] = JSD_BECKHOFF_FILTER_30000HZ;
  jsd_slave_config_.el3602.filter[1] = JSD_BECKHOFF_FILTER_30000HZ;
  return true;
}

bool fastcat::El3602::Read()
{
  jsd_el3602_read((jsd_t*)context_, slave_id_);

  const jsd_el3602_state_t* jsd_state =
      jsd_el3602_get_state((jsd_t*)context_, slave_id_);

  state_->el3602_state.voltage_ch1   = jsd_state->voltage[0];
  state_->el3602_state.voltage_ch2   = jsd_state->voltage[1];
  state_->el3602_state.adc_value_ch1 = jsd_state->adc_value[0];
  state_->el3602_state.adc_value_ch2 = jsd_state->adc_value[1];
  return true;
}

bool fastcat::El3602::RangeFromString(std::string         range_string,
                                      jsd_el3602_range_t& range)
{
  MSG("Converting range to string.");
  if (range_string.compare("10V") == 0) {
    range = JSD_EL3602_RANGE_10V;
  } else if (range_string.compare("5V") == 0) {
    range = JSD_EL3602_RANGE_5V;
  } else if (range_string.compare("2_5V") == 0) {
    range = JSD_EL3602_RANGE_2_5V;
  } else if (range_string.compare("1_25V") == 0) {
    range = JSD_EL3602_RANGE_1_25V;
  } else if (range_string.compare("75MV") == 0) {
    range = JSD_EL3602_RANGE_75MV;
  } else if (range_string.compare("200MV") == 0) {
    range = JSD_EL3602_RANGE_200MV;
  } else {
    ERROR("El3602 range %s is invalid", range_string.c_str());
    return false;
  }

  return true;
}
