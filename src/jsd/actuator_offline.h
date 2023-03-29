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
  void EgdClearErrors() override;
  void EgdFault() override;
  void EgdReset() override;
  void EgdHalt() override;
  void EgdSetPeakCurrent(double current) override;
  void EgdSetUnitMode(int32_t mode, uint16_t app_id) override;
  void EgdCSP(jsd_elmo_motion_command_csp_t jsd_csp_cmd) override;
  void EgdCSV(jsd_elmo_motion_command_csv_t jsd_csv_cmd) override;
  void EgdCST(jsd_elmo_motion_command_cst_t jsd_cst_cmd) override;
  void EgdSetGainSchedulingMode(jsd_elmo_gain_scheduling_mode_t mode,
                                uint16_t                       app_id) override;
  void EgdSetGainSchedulingIndex(uint16_t index) override;

  double  motor_on_start_time_;
  uint8_t last_motor_on_state_;
};

}  // namespace fastcat

#endif
