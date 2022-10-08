#ifndef FASTCAT_JSD_DEVICE_BASE_H_
#define FASTCAT_JSD_DEVICE_BASE_H_

// Include related header (for cc files)
#include "fastcat/device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"


namespace fastcat
{

class JsdDeviceBase: public DeviceBase
{
  public:

  // Non-pure virtual methods with default implementation
  bool WriteAsyncSdoRequest() { return true;};

  // setters/getters for EtherCat devices
  void     SetSlaveId(uint16_t slave_id);
  uint16_t GetSlaveId();
  void     SetContext(jsd_t* context);

  protected:

  jsd_t* context_ = NULL; ///< JSD context
  int    slave_id_ = 0;   ///< EtherCAT Slave Index
};

} // namespace fastcat

#endif
