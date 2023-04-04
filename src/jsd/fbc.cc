// Include related header (for cc files)
#include "fastcat/jsd/el6001.h"
#include "fastcat/jsd/fbc.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

#define HEX_FORMAT "0x%02X"

#define FBC_START_BYTE 0x53  ///< ACSII 'S'
#define FBC_STOP_BYTE  0x45  ///< ACSII 'E'
#define FBC_DATA_PACKET_SIZE 13

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
  bool ready_to_parse = false;

  jsd_el6001_read((jsd_t*)context_, slave_id_);

  const jsd_el6001_state_t* jsd_state =
      jsd_el6001_get_state((jsd_t*)context_, slave_id_);

  // Each persistent received bytes is actually a uint8_t, but the local
  // variable here uses uint32_t so that bit shifting and masking operations
  // are written out intuitively.
  uint32_t received_bytes[FBC_DATA_PACKET_SIZE] = {0};

  if (jsd_state->received_all_persistent_bytes) // assume EL6001 state is in READY_TO_COMMUNICATE
  {
    for(int i=0; i < FBC_DATA_PACKET_SIZE; i++){
      received_bytes[i] = jsd_state->persistent_received_bytes[i];
    }
    ready_to_parse = true;
  }

  if (ready_to_parse){
    // parse received bytes into FBC State
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