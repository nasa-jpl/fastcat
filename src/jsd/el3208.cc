// Include related header (for cc files)
#include "fastcat/jsd/el3208.h"

// Include c then c++ libraries
#include <string.h>

#include <algorithm>
#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El3208::El3208()
{
  MSG_DEBUG("Constructed El3208");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL3208_STATE;

  outputs_[0] = &state_->el3208_state.output_ch1;
  outputs_[1] = &state_->el3208_state.output_ch2;
  outputs_[2] = &state_->el3208_state.output_ch3;
  outputs_[3] = &state_->el3208_state.output_ch4;
  outputs_[4] = &state_->el3208_state.output_ch5;
  outputs_[5] = &state_->el3208_state.output_ch6;
  outputs_[6] = &state_->el3208_state.output_ch7;
  outputs_[7] = &state_->el3208_state.output_ch8;
}

bool fastcat::El3208::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El3208::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.driver_type          = JSD_DRIVER_TYPE_EL3208;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  // Parse element array
  YAML::Node element_node;
  if (!ParseList(node, "element", element_node)) {
    return false;
  }
  for (auto el = element_node.begin(); el != element_node.end(); ++el) {
    element_strings_.push_back((*el).as<std::string>());
  }

  // Parse connection array
  YAML::Node connection_node;
  if (!ParseList(node, "connection", connection_node)) {
    return false;
  }
  for (auto con = connection_node.begin(); con != connection_node.end();
       ++con) {
    connection_strings_.push_back((*con).as<std::string>());
  }

  // Parse wire_resistance array
  YAML::Node resistance_node;
  if (!ParseList(node, "wire_resistance", resistance_node)) {
    return false;
  }

  // Parse low_threshold array
  YAML::Node low_threshold_node;
  if (!ParseList(node, "low_threshold", low_threshold_node)) {
    return false;
  }

  // Parse high_threshold array
  YAML::Node high_threshold_node;
  if (!ParseList(node, "high_threshold", high_threshold_node)) {
    return false;
  }

  // Check the right number of entries in each array
  if (element_node.size() != JSD_EL3208_NUM_CHANNELS ||
      connection_node.size() != JSD_EL3208_NUM_CHANNELS ||
      resistance_node.size() != JSD_EL3208_NUM_CHANNELS ||
      low_threshold_node.size() != JSD_EL3208_NUM_CHANNELS ||
      high_threshold_node.size() != JSD_EL3208_NUM_CHANNELS) {
    return false;
  }

  int ii;
  for (ii = 0; ii < JSD_EL3208_NUM_CHANNELS; ii++) {
    jsd_slave_config_.el3208.presentation[ii] =
        JSD_EL3208_PRESENTATION_HIGH_RES;

    jsd_slave_config_.el3208.wire_resistance[ii] =
        resistance_node[ii].as<double>();

    jsd_slave_config_.el3208.filter[ii] = JSD_BECKHOFF_FILTER_30000HZ;

    low_threshold_[ii]  = low_threshold_node[ii].as<double>();
    high_threshold_[ii] = high_threshold_node[ii].as<double>();

    if (!ElementFromString(element_strings_[ii],
                           jsd_slave_config_.el3208.element[ii])) {
      return false;
    }

    if (!ConnectionFromString(connection_strings_[ii],
                              jsd_slave_config_.el3208.connection[ii])) {
      return false;
    }
  }

  return true;
}

bool fastcat::El3208::Read()
{
  jsd_el3208_read((jsd_t*)context_, slave_id_);

  const jsd_el3208_state_t* jsd_state =
      jsd_el3208_get_state((jsd_t*)context_, slave_id_);

  state_->el3208_state.output_ch1    = jsd_state->output_eu[0];
  state_->el3208_state.output_ch2    = jsd_state->output_eu[1];
  state_->el3208_state.output_ch3    = jsd_state->output_eu[2];
  state_->el3208_state.output_ch4    = jsd_state->output_eu[3];
  state_->el3208_state.output_ch5    = jsd_state->output_eu[4];
  state_->el3208_state.output_ch6    = jsd_state->output_eu[5];
  state_->el3208_state.output_ch7    = jsd_state->output_eu[6];
  state_->el3208_state.output_ch8    = jsd_state->output_eu[7];
  state_->el3208_state.adc_value_ch1 = jsd_state->adc_value[0];
  state_->el3208_state.adc_value_ch2 = jsd_state->adc_value[1];
  state_->el3208_state.adc_value_ch3 = jsd_state->adc_value[2];
  state_->el3208_state.adc_value_ch4 = jsd_state->adc_value[3];
  state_->el3208_state.adc_value_ch5 = jsd_state->adc_value[4];
  state_->el3208_state.adc_value_ch6 = jsd_state->adc_value[5];
  state_->el3208_state.adc_value_ch7 = jsd_state->adc_value[6];
  state_->el3208_state.adc_value_ch8 = jsd_state->adc_value[7];

  return true;
}

fastcat::FaultType fastcat::El3208::Process()
{
  if (device_fault_active_) {
    return NO_FAULT;
  }

  bool out_of_range = false;
  for (int i = 0; i < JSD_EL3208_NUM_CHANNELS; ++i) {
    if (0 == connection_strings_[i].compare("NOT_CONNECTED")) {
      continue;
    }

    if (*outputs_[i] < low_threshold_[i] || *outputs_[i] > high_threshold_[i]) {
      ERROR("EL3208 %s ch%d reads %lf, outside of permissible range(%lf, %lf)",
            name_.c_str(), i, *outputs_[i], low_threshold_[i],
            high_threshold_[i]);
      out_of_range = true;
    }
  }

  if (out_of_range) {
    return ALL_DEVICE_FAULT;
  }

  return NO_FAULT;
}

bool fastcat::El3208::ElementFromString(std::string           element_string,
                                        jsd_el3208_element_t& element)
{
  MSG_DEBUG("Parsing element string: %s", element_string.c_str());

  if (element_string.compare("PT100") == 0) {
    element = JSD_EL3208_ELEMENT_PT100;
  } else if (element_string.compare("NI100") == 0) {
    element = JSD_EL3208_ELEMENT_NI100;
  } else if (element_string.compare("PT1000") == 0) {
    element = JSD_EL3208_ELEMENT_PT1000;
  } else if (element_string.compare("PT500") == 0) {
    element = JSD_EL3208_ELEMENT_PT500;
  } else if (element_string.compare("PT200") == 0) {
    element = JSD_EL3208_ELEMENT_PT200;
  } else if (element_string.compare("NI1000") == 0) {
    element = JSD_EL3208_ELEMENT_NI1000;
  } else if (element_string.compare("NI1000_TK1500") == 0) {
    element = JSD_EL3208_ELEMENT_NI1000_TK1500;
  } else if (element_string.compare("NI120") == 0) {
    element = JSD_EL3208_ELEMENT_NI120;
  } else if (element_string.compare("OHMS4096") == 0) {
    element = JSD_EL3208_ELEMENT_OHMS4096;
  } else if (element_string.compare("OHMS1024") == 0) {
    element = JSD_EL3208_ELEMENT_OHMS1024;
  } else if (element_string.compare("KT100_ET_AL") == 0) {
    element = JSD_EL3208_ELEMENT_KT100_ET_AL;
  } else if (element_string.compare("NOT_CONNECTED") == 0) {
    // not_connected sensors handled by connection
    element = JSD_EL3208_ELEMENT_PT100;
  } else {
    ERROR("%s is not a valid element type for EL3208 devices.",
          element_string.c_str());
    return false;
  }

  return true;
}

bool fastcat::El3208::ConnectionFromString(std::string connection_string,
                                           jsd_el3208_connection_t& connection)
{
  MSG_DEBUG("Parsing connection string: %s", connection_string.c_str());

  if (connection_string.compare("2WIRE") == 0) {
    connection = JSD_EL3208_CONNECTION_2WIRE;
  } else if (connection_string.compare("3WIRE") == 0) {
    connection = JSD_EL3208_CONNECTION_3WIRE;
  } else if (connection_string.compare("4WIRE") == 0) {
    connection = JSD_EL3208_CONNECTION_4WIRE;
  } else if (connection_string.compare("NOT_CONNECTED") == 0) {
    connection = JSD_EL3208_CONNECTION_NOT_CONNECTED;
  } else {
    ERROR("%s is not a valid connection type for EL3208 devices.",
          connection_string.c_str());
    return false;
  }

  return true;
}
