#ifndef FASTCAT_ACTUATOR_OFFLINE_H_
#define FASTCAT_ACTUATOR_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/actuator.h"

namespace fastcat
{
class ActuatorOffline : public Actuator
{
 public:
  ActuatorOffline();

 protected:
  void EgdRead() override;
  void EgdSetConfig() override;
  void EgdProcess() override;
  void EgdReset() override;
  void EgdHalt() override;
  void EgdSetPeakCurrent(double current) override;
  void EgdCSP(jsd_egd_motion_command_csp_t jsd_csp_cmd) override;
  void EgdCSV(jsd_egd_motion_command_csv_t jsd_csv_cmd) override;
  void EgdCST(jsd_egd_motion_command_cst_t jsd_cst_cmd) override;
};

}  // namespace fastcat

#endif
