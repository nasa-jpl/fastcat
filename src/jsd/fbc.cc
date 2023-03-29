// Include related header (for cc files)
#include "fastcat/jsd/el6001.h"
#include "fastcat/jsd/fbc.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::Fbc::Fbc()
{
  MSG_DEBUG("Constructed Fbc");

  state_       = std::make_shared<DeviceState>();
  state_->type = FBC_STATE;
}

bool fastcat::Fbc::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::Fbc::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }

  state_->name = name_;

  jsd_slave_config_.el6001.baud_rate = JSD_EL6001_BAUD_RATE_115200;
  jsd_slave_config_.el6001.use_first_byte_as_packet_length = true;
  jsd_slave_config_.el6001.use_last_byte_as_checksum = true;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL6001_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str()); 

  return true;
}

bool fastcat::Fbc::Read()
{
  jsd_el6001_read((jsd_t*)context_, slave_id_);

  const jsd_el6001_state_t* jsd_state =
      jsd_el6001_get_state((jsd_t*)context_, slave_id_);

  state_->el6001_state.statusword   = jsd_state->statusword;
  state_->el6001_state.controlword   = jsd_state->controlword_user;

  for(int i=0; i < JSD_EL6001_NUM_DATA_BYTES; i++){
    state_->el6001_state.data_in_bytes[i] = jsd_state->received_bytes[i];
    state_->el6001_state.data_out_bytes[i] = jsd_state->transmit_bytes[i];
  }

  return true;
}

bool fastcat::Fbc::Write(DeviceCmd& cmd)
{

  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if(sdoResult != SDO_RET_VAL_NOT_APPLICABLE){
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == EL6001_WRITE_DATA_CMD) {
    uint8_t data_size = cmd.el6001_write_data_cmd.data_size;
    if (data_size > JSD_EL6001_NUM_DATA_BYTES) {
      ERROR("Data size must be in range (0,%u)", JSD_EL6001_NUM_DATA_BYTES);
      return false;
    }

    for(int i =0; i < data_size; i++){
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, i, cmd.el6001_write_data_cmd.data_out_bytes[i]);
    }    

    jsd_el6001_request_transmit_data(context_, slave_id_, data_size);
  }
  else {
    ERROR("Bad EL6001 command");
    return false;
  }

  return true;
}

fastcat::FaultType fastcat::Fbc::Process()
{
  fastcat::FaultType retval = NO_FAULT;

  jsd_el6001_process(context_, slave_id_);

  return retval;
}