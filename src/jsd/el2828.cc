// Include related header (for cc files)
#include "fastcat/jsd/el2828.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El2828::El2828()
{
  MSG_DEBUG("Constructed El2828");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL2828_STATE;
}

bool fastcat::El2828::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El2828::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.driver_type          = JSD_DRIVER_TYPE_EL2828;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El2828::Read()
{
  const jsd_el2828_state_t* jsd_state =
      jsd_el2828_get_state((jsd_t*)context_, slave_id_);

  state_->el2828_state.level_ch1 = jsd_state->output[0];
  state_->el2828_state.level_ch2 = jsd_state->output[1];
  state_->el2828_state.level_ch3 = jsd_state->output[2];
  state_->el2828_state.level_ch4 = jsd_state->output[3];
  state_->el2828_state.level_ch5 = jsd_state->output[4];
  state_->el2828_state.level_ch6 = jsd_state->output[5];
  state_->el2828_state.level_ch7 = jsd_state->output[6];
  state_->el2828_state.level_ch8 = jsd_state->output[7];

  return true;
}

fastcat::FaultType fastcat::El2828::Process()
{
  jsd_el2828_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::El2828::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == EL2828_WRITE_CHANNEL_CMD) {
    uint8_t ch = cmd.el2828_write_channel_cmd.channel;
    if (ch < 1 || ch > JSD_EL2828_NUM_CHANNELS) {
      ERROR("Channel must be in range (1,%u)", JSD_EL2828_NUM_CHANNELS);
      return false;
    }

    jsd_el2828_write_single_channel((jsd_t*)context_, slave_id_, ch - 1,
                                    cmd.el2828_write_channel_cmd.level);

  } else if (cmd.type == EL2828_WRITE_ALL_CHANNELS_CMD) {
    uint8_t output_array[JSD_EL2828_NUM_CHANNELS] = {
        cmd.el2828_write_all_channels_cmd.channel_ch1,
        cmd.el2828_write_all_channels_cmd.channel_ch2,
        cmd.el2828_write_all_channels_cmd.channel_ch3,
        cmd.el2828_write_all_channels_cmd.channel_ch4,
        cmd.el2828_write_all_channels_cmd.channel_ch5,
        cmd.el2828_write_all_channels_cmd.channel_ch6,
        cmd.el2828_write_all_channels_cmd.channel_ch7,
        cmd.el2828_write_all_channels_cmd.channel_ch8};

    jsd_el2828_write_all_channels((jsd_t*)context_, slave_id_, output_array);

  } else {
    ERROR("Bad EL2828 Command");
    return false;
  }
  return true;
}
