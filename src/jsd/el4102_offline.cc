// Include related header (for cc files)
#include "fastcat/jsd/el4102_offline.h"

// Include c then c++ libraries

// Include external then project includes

bool fastcat::El4102Offline::ConfigFromYaml(YAML::Node node,
                                            double /*external_time*/)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El4102Offline::Read() { return true; }

fastcat::FaultType fastcat::El4102Offline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::El4102Offline::Write(DeviceCmd& cmd)
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
    switch (cmd.el4102_write_channel_cmd.channel) {
      case 1:
        state_->el4102_state.voltage_output_ch1 =
            cmd.el4102_write_channel_cmd.voltage_output;
        break;
      case 2:
        state_->el4102_state.voltage_output_ch2 =
            cmd.el4102_write_channel_cmd.voltage_output;
        break;
      default:
        ERROR("Bad Channel value");
        break;
    }
  } else if (cmd.type == EL4102_WRITE_ALL_CHANNELS_CMD) {
    state_->el4102_state.voltage_output_ch1 =
        cmd.el4102_write_all_channels_cmd.voltage_output_ch1;

    state_->el4102_state.voltage_output_ch2 =
        cmd.el4102_write_all_channels_cmd.voltage_output_ch2;

  } else {
    ERROR("Bad EL4102 Command");
    return false;
  }
  return true;
}
