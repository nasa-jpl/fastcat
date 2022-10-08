// Include related header (for cc files)
#include "fastcat/jsd/jsd_device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"

void fastcat::JsdDeviceBase::SetSlaveId(uint16_t slave_id)
{
  MSG_DEBUG("Setting slave id to %u", slave_id);
  slave_id_ = slave_id;
}

uint16_t fastcat::JsdDeviceBase::GetSlaveId() { return slave_id_; }
void     fastcat::JsdDeviceBase::SetContext(jsd_t* context) { context_ = context; }
