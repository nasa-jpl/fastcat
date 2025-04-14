// Include related header (for cc files)
#include "fastcat/jsd/el2828_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

bool fastcat::El2828Offline::ConfigFromYaml(const YAML::Node& node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El2828Offline::Read() { return true; }

fastcat::FaultType fastcat::El2828Offline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::El2828Offline::Write(DeviceCmd& cmd)
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
    switch (cmd.el2828_write_channel_cmd.channel) {
      case 1:
        state_->el2828_state.level_ch1 = cmd.el2828_write_channel_cmd.level;
        break;
      case 2:
        state_->el2828_state.level_ch2 = cmd.el2828_write_channel_cmd.level;
        break;
      case 3:
        state_->el2828_state.level_ch3 = cmd.el2828_write_channel_cmd.level;
        break;
      case 4:
        state_->el2828_state.level_ch4 = cmd.el2828_write_channel_cmd.level;
        break;
      case 5:
        state_->el2828_state.level_ch5 = cmd.el2828_write_channel_cmd.level;
        break;
      case 6:
        state_->el2828_state.level_ch6 = cmd.el2828_write_channel_cmd.level;
        break;
      case 7:
        state_->el2828_state.level_ch7 = cmd.el2828_write_channel_cmd.level;
        break;
      case 8:
        state_->el2828_state.level_ch8 = cmd.el2828_write_channel_cmd.level;
        break;
      default:
        ERROR("Bad Channel value");
        break;
    }
    return true;

  } else if (cmd.type == EL2828_WRITE_ALL_CHANNELS_CMD) {
    state_->el2828_state.level_ch1 =
        cmd.el2828_write_all_channels_cmd.channel_ch1;

    state_->el2828_state.level_ch2 =
        cmd.el2828_write_all_channels_cmd.channel_ch2;

    state_->el2828_state.level_ch3 =
        cmd.el2828_write_all_channels_cmd.channel_ch3;

    state_->el2828_state.level_ch4 =
        cmd.el2828_write_all_channels_cmd.channel_ch4;

    state_->el2828_state.level_ch5 =
        cmd.el2828_write_all_channels_cmd.channel_ch5;

    state_->el2828_state.level_ch6 =
        cmd.el2828_write_all_channels_cmd.channel_ch6;

    state_->el2828_state.level_ch7 =
        cmd.el2828_write_all_channels_cmd.channel_ch7;

    state_->el2828_state.level_ch8 =
        cmd.el2828_write_all_channels_cmd.channel_ch8;

  } else {
    ERROR("Bad EL2828 Command");
    return false;
  }
  return true;
}
