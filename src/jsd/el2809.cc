// Include related header (for cc files)
#include "fastcat/jsd/el2809.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El2809::El2809()
{
  MSG_DEBUG("Constructed El2809");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL2809_STATE;
}

bool fastcat::El2809::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El2809::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.driver_type          = JSD_DRIVER_TYPE_EL2809;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El2809::Read()
{
  const jsd_el2809_state_t* jsd_state =
      jsd_el2809_get_state((jsd_t*)context_, slave_id_);

  state_->el2809_state.level_ch1 = jsd_state->output[0];
  state_->el2809_state.level_ch2 = jsd_state->output[1];
  state_->el2809_state.level_ch3 = jsd_state->output[2];
  state_->el2809_state.level_ch4 = jsd_state->output[3];
  state_->el2809_state.level_ch5 = jsd_state->output[4];
  state_->el2809_state.level_ch6 = jsd_state->output[5];
  state_->el2809_state.level_ch7 = jsd_state->output[6];
  state_->el2809_state.level_ch8 = jsd_state->output[7];
  state_->el2809_state.level_ch9 = jsd_state->output[8];
  state_->el2809_state.level_ch10 = jsd_state->output[9];
  state_->el2809_state.level_ch11 = jsd_state->output[10];
  state_->el2809_state.level_ch12 = jsd_state->output[11];
  state_->el2809_state.level_ch13 = jsd_state->output[12];
  state_->el2809_state.level_ch14 = jsd_state->output[13];
  state_->el2809_state.level_ch15 = jsd_state->output[14];
  state_->el2809_state.level_ch16 = jsd_state->output[15];

  return true;
}

fastcat::FaultType fastcat::El2809::Process()
{
  jsd_el2809_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::El2809::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == EL2809_WRITE_CHANNEL_CMD) {
    uint8_t ch = cmd.el2809_write_channel_cmd.channel;
    if (ch < 1 || ch > JSD_EL2809_NUM_CHANNELS) {
      ERROR("Channel must be in range (1,%u)", JSD_EL2809_NUM_CHANNELS);
      return false;
    }

    jsd_el2809_write_single_channel((jsd_t*)context_, slave_id_, ch - 1,
                                    cmd.el2809_write_channel_cmd.level);

  } else if (cmd.type == EL2809_WRITE_ALL_CHANNELS_CMD) {
    uint8_t output_array[JSD_EL2809_NUM_CHANNELS] = {
        cmd.el2809_write_all_channels_cmd.channel_ch1,
        cmd.el2809_write_all_channels_cmd.channel_ch2,
        cmd.el2809_write_all_channels_cmd.channel_ch3,
        cmd.el2809_write_all_channels_cmd.channel_ch4,
        cmd.el2809_write_all_channels_cmd.channel_ch5,
        cmd.el2809_write_all_channels_cmd.channel_ch6,
        cmd.el2809_write_all_channels_cmd.channel_ch7,
        cmd.el2809_write_all_channels_cmd.channel_ch8,
        cmd.el2809_write_all_channels_cmd.channel_ch9,
        cmd.el2809_write_all_channels_cmd.channel_ch10,
        cmd.el2809_write_all_channels_cmd.channel_ch11,
        cmd.el2809_write_all_channels_cmd.channel_ch12,
        cmd.el2809_write_all_channels_cmd.channel_ch13,
        cmd.el2809_write_all_channels_cmd.channel_ch14,
        cmd.el2809_write_all_channels_cmd.channel_ch15,
        cmd.el2809_write_all_channels_cmd.channel_ch16};

    jsd_el2809_write_all_channels((jsd_t*)context_, slave_id_, output_array);

  } else {
    ERROR("Bad EL2809 Command");
    return false;
  }
  return true;
}