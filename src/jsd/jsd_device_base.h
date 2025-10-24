#ifndef FASTCAT_JSD_DEVICE_BASE_H_
#define FASTCAT_JSD_DEVICE_BASE_H_

// Include related header (for cc files)
#include "fastcat/device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"

namespace fastcat
{

class JsdDeviceBase : public DeviceBase
{
 public:
  typedef enum {
    SDO_RET_VAL_FAILURE,
    SDO_RET_VAL_SUCCESS,
    SDO_RET_VAL_NOT_APPLICABLE,
  } AsyncSdoRetVal;

 public:
  // overridden DeviceBase virtual methods
  bool Write(DeviceCmd& cmd) override;

  // setters/getters for EtherCat devices
  void     SetSlaveId(uint16_t slave_id);
  uint16_t GetSlaveId();
  void     SetContext(jsd_t* context);
  void     RegisterSdoResponseQueue(
          std::shared_ptr<std::queue<SdoResponse>> sdo_response_queue);
  void SetOffline(bool is_offline);
  bool IsOffline() { return is_offline_; };

 protected:
  AsyncSdoRetVal WriteAsyncSdoRequest(DeviceCmd& cmd);
  AsyncSdoRetVal WriteAsyncSdoRequestOnline(DeviceCmd& cmd);
  AsyncSdoRetVal WriteAsyncSdoRequestOffline(DeviceCmd& cmd);

  bool   is_offline_ = false;  ///< If is an offline version
  jsd_t* context_    = NULL;   ///< JSD context
  int    slave_id_   = 0;      ///< EtherCAT Slave Index

  /// provided so that offline devices can push to this queue with
  /// SDO response stubs
  std::shared_ptr<std::queue<SdoResponse>> sdo_response_queue_;
};

}  // namespace fastcat

#endif
