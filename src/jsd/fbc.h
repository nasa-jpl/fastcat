#ifndef FASTCAT_FBC_H_
#define FASTCAT_FBC_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el6001_pub.h"

namespace fastcat
{
typedef enum {
  FBC_OP_CODE_REQUEST_DATA       = 0x01,
  FBC_OP_CODE_HB_PROTECT_ENABLE  = 0x02,  //< enable heartbeat protection at flight bluebox controller level
  FBC_OP_CODE_HB_PROTECT_DISABLE = 0x03,  //< disable heartbeat protection at flight bluebox controller level
  FBC_OP_CODE_RESET_FAULT        = 0x04,
  FBC_OP_CODE_ESTOP              = 0x05
} FbcOpCode;

typedef enum {
  FBC_BYTE_NUMBYTES = 0,
  FBC_BYTE_ADDRESS,
  FBC_BYTE_COUNTER,
  FBC_BYTE_STATUS,
  FBC_BYTE_AUX_CHANNEL,
  FBC_BYTE_ELMO_CHANNEL_1,
  FBC_BYTE_ELMO_CHANNEL_2,
  FBC_BYTE_ELMO_CHANNEL_3,
  FBC_BYTE_ELMO_CHANNEL_4,
  FBC_BYTE_ELMO_CHANNEL_5,
  FBC_BYTE_MCU_STATUS,
  FBC_BYTE_LATCHED_FAULT,
  FBC_BYTE_CHECKSUM
} FbcResponseBytes;

typedef enum {
  FBC_STATUS_BIT_SAFETY_RELAY_ENABLED = 0,
  FBC_STATUS_BIT_EFUSES_ENABLED,
  FBC_STATUS_BIT_HEARTBEAT_ENABLED,
  FBC_STATUS_BIT_HEARTBEAT_1,
  FBC_STATUS_BIT_HEARTBEAT_2,
  FBC_STATUS_BIT_HEARTBEAT_3,
  FBC_STATUS_BIT_HEARTBEAT_4,
  FBC_STATUS_BIT_HEARTBEAT_5,
} FbcResponseBitsStatus;

typedef enum {
  FBC_AUX_BIT_SGAUGE_EFUSE = 0,
  FBC_AUX_BIT_HEATER_1_EFUSE,
  FBC_AUX_BIT_HEATER_2_EFUSE,
  FBC_AUX_BIT_RESOLVER_ERROR,
  FBC_AUX_BIT_MR_SENSOR_EFUSE,
} FbcResponseBitsAux;

typedef enum {
  FBC_ELMO_BIT_ELMO_EFUSE = 0,
  FBC_ELMO_BIT_HALL_EFUSE,
  FBC_ELMO_BIT_RESOLVER_ERROR,
  FBC_ELMO_BIT_HEATER_1_EFUSE,
  FBC_ELMO_BIT_HEATER_2_EFUSE,
  FBC_ELMO_BIT_BRAKE_1_EFUSE,
  FBC_ELMO_BIT_BRAKE_2_EFUSE,
} FbcResponseBitsElmo;

typedef enum {
  FBC_AUX_CHANNEL = 0,
  FBC_M1_CHANNEL,
  FBC_M2_CHANNEL,
  FBC_M3_CHANNEL,
  FBC_M4_CHANNEL,
  FBC_M5_CHANNEL,
  FBC_NUM_CHANNELS
} FbcChannels;

typedef enum {
  FBC_STATUS_ALL_OK = 0,
  FBC_STATUS_FAULTED,
  FBC_STATUS_STANDBY,
  FBC_STATUS_HB_BYPASSED,

} FbcStatus;

class Fbc : public JsdDeviceBase
{
 public:
  Fbc();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
  void      Fault() override;
  void      Reset() override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);
  
  // virtual void EgdRead();
  // virtual void EgdSetConfig();
  // virtual void EgdProcess();
  // virtual void EgdClearErrors();
  // virtual void EgdFault();
  // virtual void EgdReset();
    
  jsd_el6001_state_t jsd_el6001_state_;
  DeviceCmd          last_cmd_;

 private:
  jsd_slave_config_t jsd_slave_config_;
};

}  // namespace fastcat

#endif
