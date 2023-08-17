// Include related header (for cc files)
#include "fastcat/jsd/el2809_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

bool fastcat::El2809Offline::ConfigFromYaml(YAML::Node node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El2809Offline::Read() { return true; }

fastcat::FaultType fastcat::El2809Offline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::El2809Offline::Write(DeviceCmd& cmd)
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
    switch (cmd.el2809_write_channel_cmd.channel) {
      case 1:
        state_->el2809_state.level_ch1 = cmd.el2809_write_channel_cmd.level;
        break;
      case 2:
        state_->el2809_state.level_ch2 = cmd.el2809_write_channel_cmd.level;
        break;
      case 3:
        state_->el2809_state.level_ch3 = cmd.el2809_write_channel_cmd.level;
        break;
      case 4:
        state_->el2809_state.level_ch4 = cmd.el2809_write_channel_cmd.level;
        break;
      case 5:
        state_->el2809_state.level_ch5 = cmd.el2809_write_channel_cmd.level;
        break;
      case 6:
        state_->el2809_state.level_ch6 = cmd.el2809_write_channel_cmd.level;
        break;
      case 7:
        state_->el2809_state.level_ch7 = cmd.el2809_write_channel_cmd.level;
        break;
      case 8:
        state_->el2809_state.level_ch8 = cmd.el2809_write_channel_cmd.level;
        break;
      case 9:
        state_->el2809_state.level_ch9 = cmd.el2809_write_channel_cmd.level;
        break;
      case 10:
        state_->el2809_state.level_ch10 = cmd.el2809_write_channel_cmd.level;
        break;
      case 11:
        state_->el2809_state.level_ch11 = cmd.el2809_write_channel_cmd.level;
        break;
      case 12:
        state_->el2809_state.level_ch12 = cmd.el2809_write_channel_cmd.level;
        break;
      case 13:
        state_->el2809_state.level_ch13 = cmd.el2809_write_channel_cmd.level;
        break;
      case 14:
        state_->el2809_state.level_ch14 = cmd.el2809_write_channel_cmd.level;
        break;
      case 15:
        state_->el2809_state.level_ch15 = cmd.el2809_write_channel_cmd.level;
        break;
      case 16:
        state_->el2809_state.level_ch16 = cmd.el2809_write_channel_cmd.level;
        break;
      default:
        ERROR("Bad Channel value");
        break;
    }
    return true;

  } else if (cmd.type == EL2809_WRITE_ALL_CHANNELS_CMD) {
    state_->el2809_state.level_ch1 =
        cmd.el2809_write_all_channels_cmd.channel_ch1;

    state_->el2809_state.level_ch2 =
        cmd.el2809_write_all_channels_cmd.channel_ch2;

    state_->el2809_state.level_ch3 =
        cmd.el2809_write_all_channels_cmd.channel_ch3;

    state_->el2809_state.level_ch4 =
        cmd.el2809_write_all_channels_cmd.channel_ch4;
    
    state_->el2809_state.level_ch5 =
        cmd.el2809_write_all_channels_cmd.channel_ch5;

    state_->el2809_state.level_ch6 =
        cmd.el2809_write_all_channels_cmd.channel_ch6;

    state_->el2809_state.level_ch7 =
        cmd.el2809_write_all_channels_cmd.channel_ch7;

    state_->el2809_state.level_ch8 =
        cmd.el2809_write_all_channels_cmd.channel_ch8;

    state_->el2809_state.level_ch9 =
        cmd.el2809_write_all_channels_cmd.channel_ch9;

    state_->el2809_state.level_ch10 =
        cmd.el2809_write_all_channels_cmd.channel_ch10;

    state_->el2809_state.level_ch11 =
        cmd.el2809_write_all_channels_cmd.channel_ch11;

    state_->el2809_state.level_ch12 =
        cmd.el2809_write_all_channels_cmd.channel_ch12;

    state_->el2809_state.level_ch13 =
        cmd.el2809_write_all_channels_cmd.channel_ch13;

    state_->el2809_state.level_ch14 =
        cmd.el2809_write_all_channels_cmd.channel_ch14;

    state_->el2809_state.level_ch15 =
        cmd.el2809_write_all_channels_cmd.channel_ch15;

    state_->el2809_state.level_ch16 =
        cmd.el2809_write_all_channels_cmd.channel_ch16;

  } else {
    ERROR("Bad EL2809 Command");
    return false;
  }
  return true;
}
