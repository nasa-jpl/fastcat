// Include related header (for cc files)
#include "fastcat/jsd/el4102.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El4102::El4102()
{
  MSG_DEBUG("Constructed El4102");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL4102_STATE;
}

bool fastcat::El4102::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El4102::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL4102_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El4102::Read()
{
  const jsd_el4102_state_t* jsd_state =
      jsd_el4102_get_state((jsd_t*)context_, slave_id_);

  state_->el4102_state.voltage_output_ch1 = jsd_state->voltage_output[0];
  state_->el4102_state.voltage_output_ch2 = jsd_state->voltage_output[1];

  return true;
}

fastcat::FaultType fastcat::El4102::Process()
{
  jsd_el4102_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::El4102::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  // The use of ERROR statements, which are not real-time safe, is deemed
  // acceptable in this function. Their execution would imply the existence of
  // a serious issue in the application's side that should be addressed.
  if (cmd.type == EL4102_WRITE_CHANNEL_CMD) {
    uint8_t ch = cmd.el4102_write_channel_cmd.channel;
    if (ch < 1 || ch > JSD_EL4102_NUM_CHANNELS) {
      ERROR("Channel must be in range (1,%u)", JSD_EL4102_NUM_CHANNELS);
      return false;
    }

    jsd_el4102_write_single_channel(
        context_, slave_id_, ch - 1,
        cmd.el4102_write_channel_cmd.voltage_output);

  } else if (cmd.type == EL4102_WRITE_ALL_CHANNELS_CMD) {
    double output_array[JSD_EL4102_NUM_CHANNELS] = {
        cmd.el4102_write_all_channels_cmd.voltage_output_ch1,
        cmd.el4102_write_all_channels_cmd.voltage_output_ch2};

    jsd_el4102_write_all_channels((jsd_t*)context_, slave_id_, output_array);
  } else {
    ERROR("Bad EL4102 command");
    return false;
  }
  return true;
}
