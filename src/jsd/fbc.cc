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

#define FBC_NUM_HEATER_PER_CHANNEL 2
#define FBC_NUM_BRAKE_PER_ELMO 2

fastcat::Fbc::Fbc()
{
  MSG_DEBUG("Constructed Fbc");

  state_       = std::make_shared<DeviceState>();
  state_->type = FBC_STATE;

  counter = 0;
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
    state_->fbc_state.is_data_available = true;    
    // parse received bytes into FBC State
    for (int i = FBC_BYTE_COUNTER; i <= FBC_BYTE_LATCHED_FAULT; i++) {
      // Copy over the received bytes for telem and diagnostics
      switch(i)
      {
        case FBC_BYTE_COUNTER:
          state_->fbc_state.raw_byte_02 = received_bytes[i];
          break;
        case FBC_BYTE_STATUS:
          state_->fbc_state.raw_byte_03 = received_bytes[i];
          break;
        case FBC_BYTE_AUX_CHANNEL:
          state_->fbc_state.raw_byte_04 = received_bytes[i];
          break;
        case FBC_BYTE_ELMO_CHANNEL_1:
          state_->fbc_state.raw_byte_05 = received_bytes[i];
          break;
        case FBC_BYTE_ELMO_CHANNEL_2:
          state_->fbc_state.raw_byte_06 = received_bytes[i];
          break;
        case FBC_BYTE_ELMO_CHANNEL_3:
          state_->fbc_state.raw_byte_07 = received_bytes[i];
          break;
        case FBC_BYTE_ELMO_CHANNEL_4:
          state_->fbc_state.raw_byte_08 = received_bytes[i];
          break;
        case FBC_BYTE_ELMO_CHANNEL_5:
          state_->fbc_state.raw_byte_09 = received_bytes[i];
          break;
        case FBC_BYTE_MCU_STATUS:
          state_->fbc_state.raw_byte_10 = received_bytes[i];
          break;
        case FBC_BYTE_LATCHED_FAULT:
          state_->fbc_state.raw_byte_11 = received_bytes[i];
          break;
        default:
          break;
      }

      // Set the downstream (derived) variables
      switch(i)
      {
        case FBC_BYTE_COUNTER:
          break;

        case FBC_BYTE_STATUS:          
          state_->fbc_state.safety_relay_enabled               = ( received_bytes[i]       & 0x01);
          state_->fbc_state.efuses_enabled                     = ((received_bytes[i] >> 1) & 0x01);
          state_->fbc_state.heartbeat_protection_enabled       = ((received_bytes[i] >> 2) & 0x01);
          state_->fbc_state.heartbeat_detected[FBC_M1_CHANNEL] = ((received_bytes[i] >> 3) & 0x01);
          state_->fbc_state.heartbeat_detected[FBC_M2_CHANNEL] = ((received_bytes[i] >> 4) & 0x01);
          state_->fbc_state.heartbeat_detected[FBC_M3_CHANNEL] = ((received_bytes[i] >> 5) & 0x01);
          state_->fbc_state.heartbeat_detected[FBC_M4_CHANNEL] = ((received_bytes[i] >> 6) & 0x01);
          state_->fbc_state.heartbeat_detected[FBC_M5_CHANNEL] = ((received_bytes[i] >> 7) & 0x01);          
          break;

        case FBC_BYTE_AUX_CHANNEL:
          state_->fbc_state.straingauge_efuse_ok  = ( received_bytes[i]       & 0x01);
          state_->fbc_state.heater_efuse_ok[0][0] = ((received_bytes[i] >> 1) & 0x01);
          state_->fbc_state.heater_efuse_ok[0][1] = ((received_bytes[i] >> 1) & 0x01);
          state_->fbc_state.resolver_efuse_ok[0]  = ((received_bytes[i] >> 3) & 0x01);
          state_->fbc_state.mr_sensor_efuse_ok    = ((received_bytes[i] >> 4) & 0x01);          
          break;

        case FBC_BYTE_ELMO_CHANNEL_1:
        case FBC_BYTE_ELMO_CHANNEL_2:
        case FBC_BYTE_ELMO_CHANNEL_3:
        case FBC_BYTE_ELMO_CHANNEL_4:
        case FBC_BYTE_ELMO_CHANNEL_5:
          {
            int elmo_index = (i - FBC_BYTE_ELMO_CHANNEL_1 + FBC_M1_CHANNEL);
            state_->fbc_state.elmo_efuse_ok[elmo_index]      = ( received_bytes[i]       & 0x01);
            state_->fbc_state.hall_efuse_ok[elmo_index]      = ((received_bytes[i] >> 1) & 0x01);
            state_->fbc_state.resolver_efuse_ok[elmo_index]  = ((received_bytes[i] >> 2) & 0x01);
            state_->fbc_state.heater_efuse_ok[elmo_index][0] = ((received_bytes[i] >> 3) & 0x01);
            state_->fbc_state.heater_efuse_ok[elmo_index][1] = ((received_bytes[i] >> 4) & 0x01);
            state_->fbc_state.brake_efuse_ok[elmo_index][0]  = ((received_bytes[i] >> 5) & 0x01);
            state_->fbc_state.brake_efuse_ok[elmo_index][1]  = ((received_bytes[i] >> 6) & 0x01);
          }
          break;

        case FBC_BYTE_MCU_STATUS:
          // Pull out bits 0 and 1 (0000 0011)
          state_->fbc_state.fbc_sms = (FbcSms) (received_bytes[i] & 0x03);
          break;

        case FBC_BYTE_LATCHED_FAULT:
          state_->fbc_state.latched_fault_bit  = ( received_bytes[i]       & 0x0F); //0000 1111
          state_->fbc_state.latched_fault_byte = ((received_bytes[i] >> 4) & 0x0F); //1111 0000
          break;

        default:
          ERROR("Unhandled response byte for efuse mcu");
          break;
      }
    }
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

  switch (cmd.type) {
    case FBC_REQUEST_DATA_CMD:
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 0, 0x53);     // Byte 0: Start character "S"
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 1, 0x00);     // Byte 1: Arduino Address
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 2, 0x01);     // Byte 2: OpCode: Request Data
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 3, counter);  // Byte 3: counter
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 4, 0);        // Byte 4: checksum
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 5, 0x45);     // Byte 5: End character "E"
      
      jsd_el6001_request_transmit_data(context_, slave_id_, 6);
      break;

    case FBC_RESET_FAULT_CMD:
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 0, 0x53);     // Byte 0: Start character "S"
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 1, 0x00);     // Byte 1: Arduino Address
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 2, 0x04);     // Byte 2: OpCode: Reset fault
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 3, counter);  // Byte 3: counter
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 4, 0);        // Byte 4: checksum
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 5, 0x45);     // Byte 5: End character "E"
      
      jsd_el6001_request_transmit_data(context_, slave_id_, 6);
      break;

    case FBC_TRIGGER_ESTOP_CMD:
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 0, 0x53);     // Byte 0: Start character "S"
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 1, 0x00);     // Byte 1: Arduino Address
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 2, 0x05);     // Byte 2: OpCode: Trigger ESTOP
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 3, counter);  // Byte 3: counter
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 4, 0);        // Byte 4: checksum
      jsd_el6001_set_transmit_data_8bits(context_, slave_id_, 5, 0x45);     // Byte 5: End character "E"
      
      jsd_el6001_request_transmit_data(context_, slave_id_, 6);
      break;

    default:
      ERROR("Bad FBC command");
      break;
  }

  counter++;

  return true;
}

fastcat::FaultType fastcat::Fbc::Process()
{
  fastcat::FaultType retval = NO_FAULT;

  jsd_el6001_process(context_, slave_id_);

  return retval;
}

void fastcat::Fbc::Fault()
{
  WARNING("Faulting FBC %s", name_.c_str());
  // ElmoFault(); //TODO: Invoke FBC fault

}

void fastcat::Fbc::Reset()
{
  WARNING("Resetting FBC device %s", name_.c_str());
  // TODO: Invoke Reset FBC
  // if (actuator_sms_ == ACTUATOR_SMS_FAULTED) {
  //   // Resetting here would open brakes so we explicitly do not reset the EGD
  //   // and instead only clear latched errors
  //   ElmoClearErrors();
  //   fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_OKAY;
  //   TransitionToState(ACTUATOR_SMS_HALTED);
  // }
}