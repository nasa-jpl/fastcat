// Include related header (for cc files)
#include "fastcat/jsd/el2124.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El2124::El2124()
{
  MSG_DEBUG("Constructed El2124");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL2124_STATE;
}

bool fastcat::El2124::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El2124::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL2124_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El2124::Read()
{
  const jsd_el2124_state_t* jsd_state =
      jsd_el2124_get_state((jsd_t*)context_, slave_id_);

  state_->el2124_state.level_ch1 = jsd_state->output[0];
  state_->el2124_state.level_ch2 = jsd_state->output[1];
  state_->el2124_state.level_ch3 = jsd_state->output[2];
  state_->el2124_state.level_ch4 = jsd_state->output[3];

  return true;
}

fastcat::FaultType fastcat::El2124::Process()
{
  jsd_el2124_process((jsd_t*)context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::El2124::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == EL2124_WRITE_CHANNEL_CMD) {
    uint8_t ch = cmd.el2124_write_channel_cmd.channel;
    if (ch < 1 || ch > JSD_EL2124_NUM_CHANNELS) {
      ERROR("Channel must be in range (1,%u)", JSD_EL2124_NUM_CHANNELS);
      return false;
    }

    jsd_el2124_write_single_channel((jsd_t*)context_, slave_id_, ch - 1,
                                    cmd.el2124_write_channel_cmd.level);

  } else if (cmd.type == EL2124_WRITE_ALL_CHANNELS_CMD) {
    uint8_t output_array[JSD_EL2124_NUM_CHANNELS] = {
        cmd.el2124_write_all_channels_cmd.channel_ch1,
        cmd.el2124_write_all_channels_cmd.channel_ch2,
        cmd.el2124_write_all_channels_cmd.channel_ch3,
        cmd.el2124_write_all_channels_cmd.channel_ch4};

    jsd_el2124_write_all_channels((jsd_t*)context_, slave_id_, output_array);

  } else {
    ERROR("Bad EL2124 Command");
    return false;
  }
  return true;
}
