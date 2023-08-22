#ifndef FASTCAT_PLATINUM_ACTUATOR_OFFLINE_H_
#define FASTCAT_PLATINUM_ACTUATOR_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/platinum_actuator.h"

namespace fastcat
{
class PlatinumActuatorOffline : public PlatinumActuator
{
 public:
  PlatinumActuatorOffline();
  bool ConfigFromYaml(YAML::Node node, double external_time = -1) override;

 private:
  bool HandleNewProfPosCmdImpl(const DeviceCmd& cmd) override;
  bool HandleNewProfVelCmdImpl(const DeviceCmd& cmd) override;
  bool HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd) override;

  FaultType ProcessProfPos() override;
  FaultType ProcessProfVel() override;
  FaultType ProcessProfTorque() override;
  FaultType ProcessProfPosDisengaging() override;
  FaultType ProcessProfVelDisengaging() override;
  FaultType ProcessProfTorqueDisengaging() override;

  void ElmoSetConfig() override;
  void ElmoProcess() override;
  void ElmoFault() override;
  void ElmoReset() override;
  void ElmoRead() override;
  void ElmoClearErrors() override;
  void ElmoSetPeakCurrent(double current) override;
  void ElmoSetUnitMode(int32_t mode, uint16_t app_id) override;
  void ElmoCSP(const jsd_elmo_motion_command_csp_t& jsd_csp_cmd) override;
  void ElmoCSV(const jsd_elmo_motion_command_csv_t& jsd_csv_cmd) override;
  void ElmoCST(const jsd_elmo_motion_command_cst_t& jsd_cst_cmd) override;
  void ElmoSetGainSchedulingMode(jsd_elmo_gain_scheduling_mode_t mode,
                                 uint16_t app_id) override;
  void ElmoSetGainSchedulingIndex(uint16_t index) override;
  void ElmoHalt() override;

  double  motor_on_start_time_ = 0.0;
  uint8_t last_motor_on_state_ = 0;
};

}  // namespace fastcat

#endif
