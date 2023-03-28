#ifndef FASTCAT_COMMANDER_H_
#define FASTCAT_COMMANDER_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
class Commander : public FastcatDeviceBase
{
 public:
  Commander();
  bool Initialize() override;
  bool Read() override;
  bool Write(DeviceCmd& cmd) override;
  void Fault() override;
  void Reset() override;

 protected:
  DeviceCmd device_cmd_;
  double    enable_time_ = 0;
  double    enable_duration_ = 1.0;
  uint16_t  skip_counter_ = 0;
};

}  // namespace fastcat

#endif
