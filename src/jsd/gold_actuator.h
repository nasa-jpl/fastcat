#ifndef FASTCAT_GOLD_ACTUATOR_H_
#define FASTCAT_GOLD_ACTUATOR_H_

// Include related header (for cc files)

// Include C then C++ libraries

// Include external then project includes
#include "fastcat/jsd/actuator.h"
#include "jsd/jsd_egd_pub.h"

namespace fastcat
{
class GoldActuator : public Actuator
{
  friend class Tester;

 public:
  GoldActuator();

 protected:
  jsd_egd_state_t jsd_egd_state_ = {};

 private:
  void PopulateJsdSlaveConfig() override;
  void PopulateState() override;

  bool HandleNewProfPosCmdImpl(const DeviceCmd& cmd) override;
  bool HandleNewProfVelCmdImpl(const DeviceCmd& cmd) override;
  bool HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd) override;

  FaultType ProcessProfPos() override;
  FaultType ProcessProfVel() override;
  FaultType ProcessProfTorque() override;
  FaultType ProcessProfPosDisengaging() override;
  FaultType ProcessProfVelDisengaging() override;
  FaultType ProcessProfTorqueDisengaging() override;

  void ElmoRead() override;
  void ElmoClearErrors() override;
  void ElmoSetPeakCurrent(double current) override;
  void ElmoSetDigitalOutput(uint8_t digital_output_index,
                            uint8_t output_level) override;
  void ElmoSetUnitMode(int32_t mode, uint16_t app_id) override;
  void ElmoSetGainSchedulingMode(jsd_elmo_gain_scheduling_mode_t mode,
                                 uint16_t app_id) override;
  void ElmoSetGainSchedulingIndex(uint16_t index) override;
  void ElmoFault() override;
  void ElmoReset() override;
  void ElmoCSP(const jsd_elmo_motion_command_csp_t& jsd_csp_cmd) override;
  void ElmoCSV(const jsd_elmo_motion_command_csv_t& jsd_csv_cmd) override;
  void ElmoCST(const jsd_elmo_motion_command_cst_t& jsd_cst_cmd) override;
  void ElmoHalt() override;
  void ElmoProcess() override;

  double                         GetActualVelocity() override;
  double                         GetElmoActualPosition() override;
  jsd_elmo_state_machine_state_t GetElmoStateMachineState() override;
  bool                           IsStoEngaged() override;
};

}  // namespace fastcat

#endif
