#ifndef FASTCAT_EL2124_H_
#define FASTCAT_EL2124_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el2124_pub.h"

namespace fastcat
{
class El2124 : public JsdDeviceBase
{
 public:
  El2124();
  bool      Initialize() override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;

 protected:
  bool InitializeCommon();

 private:
  jsd_slave_config_t jsd_slave_config_ = {0};
};

}  // namespace fastcat

#endif
