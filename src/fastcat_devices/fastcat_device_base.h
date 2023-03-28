#ifndef FASTCAT_FASTCAT_DEVICE_BASE_H_
#define FASTCAT_FASTCAT_DEVICE_BASE_H_

// Include related header (for cc files)
#include "fastcat/device_base.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"

namespace fastcat
{

class FastcatDeviceBase : public DeviceBase
{
public:
  std::vector<Signal> signals_;
}

}  // namespace fastcat

#endif
