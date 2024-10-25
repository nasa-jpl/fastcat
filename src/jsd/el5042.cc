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

  YAML::Node invert_feedback_direction_node;
  if (!ParseList(node, "invert_feedback_direction", invert_feedback_direction_node)) {
    return false;
  }

  YAML::Node disable_status_bits_node;
  if (!ParseList(node, "disable_status_bits", disable_status_bits_node)) {
    return false;
  }

  YAML::Node invert_checksum_node;
  if (!ParseList(node, "invert_checksum", invert_checksum_node)) {
    return false;
  }

  YAML::Node checksum_polynomial_node;
  if (!ParseList(node, "checksum_polynomial", checksum_polynomial_node)) {
    return false;
  }

  YAML::Node supply_voltage_node;
  if (!ParseList(node, "supply_voltage", supply_voltage_node)) {
    return false;
  }

  YAML::Node clock_frequency_node;
  if (!ParseList(node, "clock_frequency", clock_frequency_node)) {
    return false;
  }

  YAML::Node gray_code_node;
  if (!ParseList(node, "gray_code", gray_code_node)) {
    return false;
  }

  YAML::Node multiturn_bits_node;
  if (!ParseList(node, "multiturn_bits", multiturn_bits_node)) {
    return false;
  }

  YAML::Node singleturn_bits_node;
  if (!ParseList(node, "singleturn_bits", singleturn_bits_node)) {
    return false;
  }

  YAML::Node offset_bits_node;
  if (!ParseList(node, "offset_bits", offset_bits_node)) {
    return false;
  }

  YAML::Node ssi_mode_node;
  if (!ParseList(node, "ssi_mode_bits", ssi_mode_node)) {
    return false;
  }

  // Check the right number of entries in each array
  if (invert_feedback_direction_node.size() != JSD_EL5042_NUM_CHANNELS ||
      disable_status_bits_node.size() != JSD_EL5042_NUM_CHANNELS ||
      invert_checksum_node.size() != JSD_EL5042_NUM_CHANNELS ||
      checksum_polynomial_node.size() != JSD_EL5042_NUM_CHANNELS ||
      supply_voltage_node.size() != JSD_EL5042_NUM_CHANNELS ||
      clock_frequency_node.size() != JSD_EL5042_NUM_CHANNELS ||
      gray_code_node.size() != JSD_EL5042_NUM_CHANNELS ||
      multiturn_bits_node.size() != JSD_EL5042_NUM_CHANNELS ||
      singleturn_bits_node.size() != JSD_EL5042_NUM_CHANNELS ||
      offset_bits_node.size() != JSD_EL5042_NUM_CHANNELS ||
      ssi_mode_node.size() != JSD_EL5042_NUM_CHANNELS) {
    return false;
  }

  int ii;
  for (ii = 0; ii < JSD_EL3208_NUM_CHANNELS; ii++) {
    jsd_slave_config_.el5042.invert_feedback_direction[ii] =
        invert_feedback_direction_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.disable_status_bits[ii] =
        disable_status_bits_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.invert_checksum[ii] =
        invert_checksum_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.checksum_polynomial[ii] =
        checksum_polynomial_node[ii].as<uint32_t>();

    jsd_slave_config_.el5042.supply_voltage[ii] =
        supply_voltage_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.clock_frequency[ii] =
        clock_frequency_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.gray_code[ii] =
        gray_code_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.multiturn_bits[ii] =
        multiturn_bits_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.singleturn_bits[ii] =
        singleturn_bits_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.offset_bits[ii] =
        offset_bits_node[ii].as<uint8_t>();

    jsd_slave_config_.el5042.ssi_mode[ii] =
        ssi_mode_node[ii].as<uint8_t>();
  }

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
