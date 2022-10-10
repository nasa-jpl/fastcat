// Include related header (for cc files)
#include "fastcat/jsd/jsd_device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_sdo_pub.h"
#include "jsd/jsd_print.h"

bool fastcat::JsdDeviceBase::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if(sdoResult != SDO_RET_VAL_NOT_APPLICABLE){
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  return true;
}

void fastcat::JsdDeviceBase::SetSlaveId(uint16_t slave_id)
{
  MSG_DEBUG("Setting slave id to %u", slave_id);
  slave_id_ = slave_id;
}

uint16_t fastcat::JsdDeviceBase::GetSlaveId() { 
  return slave_id_; 
}

void fastcat::JsdDeviceBase::SetContext(jsd_t* context) { 
  context_ = context; 
}

void fastcat::JsdDeviceBase::RegisterSdoResponseQueue(
    std::shared_ptr<std::queue<SdoResponse>> sdo_response_queue)
{
  sdo_response_queue_ = sdo_response_queue;
}

void fastcat::JsdDeviceBase::SetOffline(bool is_offline){
  is_offline_ = is_offline;
}

fastcat::JsdDeviceBase::AsyncSdoRetVal fastcat::JsdDeviceBase::WriteAsyncSdoRequest(DeviceCmd& cmd){
  return (is_offline_) ? WriteAsyncSdoRequestOffline(cmd) : WriteAsyncSdoRequestOnline(cmd);
}

fastcat::JsdDeviceBase::AsyncSdoRetVal fastcat::JsdDeviceBase::WriteAsyncSdoRequestOnline(DeviceCmd& cmd){

  AsyncSdoRetVal retval = SDO_RET_VAL_NOT_APPLICABLE;

  if(cmd.type == ASYNC_SDO_WRITE_CMD){
    
    if(jsd_sdo_set_param_async(context_, slave_id_, 
         cmd.async_sdo_write_cmd.sdo_index,
         cmd.async_sdo_write_cmd.sdo_subindex,
         cmd.async_sdo_write_cmd.data_type,
         static_cast<void*>(&cmd.async_sdo_write_cmd.data),
         cmd.async_sdo_write_cmd.app_id))
    {
      retval = SDO_RET_VAL_SUCCESS;
    }else{
      retval = SDO_RET_VAL_FAILURE;
    }

  }else if(cmd.type == ASYNC_SDO_READ_CMD){

    if(jsd_sdo_get_param_async(context_, slave_id_, 
         cmd.async_sdo_read_cmd.sdo_index,
         cmd.async_sdo_read_cmd.sdo_subindex,
         cmd.async_sdo_read_cmd.data_type,
         cmd.async_sdo_read_cmd.app_id))
    {  
      retval = SDO_RET_VAL_SUCCESS;
    }else{
      retval = SDO_RET_VAL_FAILURE;
    }
  }
  return retval;
}

fastcat::JsdDeviceBase::AsyncSdoRetVal fastcat::JsdDeviceBase::WriteAsyncSdoRequestOffline(DeviceCmd& cmd){

  AsyncSdoRetVal retval = SDO_RET_VAL_NOT_APPLICABLE;

  SdoResponse resp;

  if(cmd.type == ASYNC_SDO_WRITE_CMD){

    resp.device_name           = cmd.name;
    resp.response.request_type = JSD_SDO_REQ_TYPE_WRITE;
    resp.response.slave_id     = slave_id_;
    resp.response.sdo_index    = cmd.async_sdo_write_cmd.sdo_index;
    resp.response.sdo_subindex = cmd.async_sdo_write_cmd.sdo_subindex;
    resp.response.data         = cmd.async_sdo_write_cmd.data;
    resp.response.data_type    = cmd.async_sdo_write_cmd.data_type;
    resp.response.app_id       = cmd.async_sdo_write_cmd.app_id;

    sdo_response_queue_->push(resp);
    retval = SDO_RET_VAL_SUCCESS;

  }else if(cmd.type == ASYNC_SDO_READ_CMD){

    resp.device_name           = cmd.name;
    resp.response.request_type = JSD_SDO_REQ_TYPE_READ;
    resp.response.slave_id     = slave_id_;
    resp.response.sdo_index    = cmd.async_sdo_read_cmd.sdo_index;
    resp.response.sdo_subindex = cmd.async_sdo_read_cmd.sdo_subindex;
    resp.response.data         = {0};
    resp.response.data_type    = cmd.async_sdo_read_cmd.data_type;
    resp.response.app_id       = cmd.async_sdo_read_cmd.app_id;

    sdo_response_queue_->push(resp);
    retval = SDO_RET_VAL_SUCCESS;
  }

  return retval;
}
