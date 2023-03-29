// Include related header (for cc files)
#include "fastcat/jsd/fbc_offline.h"

// Include c then c++ libraries

// Include external then project includes

bool fastcat::FbcOffline::ConfigFromYaml(YAML::Node node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::FbcOffline::Read() { return true; }

fastcat::FaultType fastcat::FbcOffline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::El4102Offline::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if(sdoResult != SDO_RET_VAL_NOT_APPLICABLE){
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  // if (cmd.type == EL6001_WRITE_DATA_CMD) {    
  //   uint8_t data_size = cmd.el6001_write_data_cmd.data_size;
  //   if (data_size > JSD_EL6001_NUM_DATA_BYTES) {
  //     ERROR("Data size must be in range (0,%u)", JSD_EL6001_NUM_DATA_BYTES);
  //     return false;
  //   }
    
  //   for(int i = 0; i < data_size; i++){
  //     state_->el6001_state.data_out_bytes[i] = cmd.el6001_write_data_cmd.data_out_bytes[i];
  //   }    

  // } else {
  //   ERROR("Bad EL6001 Command");
  //   return false;
  // }
  return true;
}
