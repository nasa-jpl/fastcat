#ifndef FASTCAT_ACTUATOR_H_
#define FASTCAT_ACTUATOR_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "fastcat/trap.h"
#include "jsd/jsd_egd_pub.h"

namespace fastcat
{
typedef enum {
  ACTUATOR_SMS_FAULTED,
  ACTUATOR_SMS_HALTED,
  ACTUATOR_SMS_HOLDING,
  ACTUATOR_SMS_PROF_POS,
  ACTUATOR_SMS_PROF_POS_DISENGAGING,
  ACTUATOR_SMS_PROF_VEL,
  ACTUATOR_SMS_PROF_VEL_DISENGAGING,
  ACTUATOR_SMS_PROF_TORQUE,
  ACTUATOR_SMS_PROF_TORQUE_DISENGAGING,
  ACTUATOR_SMS_CS,
  ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP,
  ACTUATOR_SMS_CAL_AT_HARDSTOP,
  ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP,
} ActuatorStateMachineState;

typedef enum {
  ACTUATOR_TYPE_REVOLUTE,
  ACTUATOR_TYPE_PRISMATIC,
} ActuatorType;

class Actuator : public JsdDeviceBase
{
 public:
  Actuator();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
  void      Fault() override;
  void      Reset() override;
  bool      SetOutputPosition(double position);
  bool      HasAbsoluteEncoder();

 protected:
  double  CntsToEu(int32_t cnts);
  int32_t EuToCnts(double eu);
  double  PosCntsToEu(int32_t cnts);
  int32_t PosEuToCnts(double eu);

  bool PosExceedsCmdLimits(double pos_eu);
  bool VelExceedsCmdLimits(double vel_eu);
  bool AccExceedsCmdLimits(double vel_eu);
  bool CurrentExceedsCmdLimits(double current);

  void               RequestStateMachineState(ActuatorStateMachineState sms);
  void               TransitionToState(ActuatorStateMachineState sms);
  static std::string StateMachineStateToString(ActuatorStateMachineState sms);

  bool CheckStateMachineMotionCmds();
  bool CheckStateMachineGainSchedulingCmds();

  bool HandleNewCSPCmd(DeviceCmd& cmd);
  bool HandleNewCSVCmd(DeviceCmd& cmd);
  bool HandleNewCSTCmd(DeviceCmd& cmd);
  bool HandleNewProfPosCmd(DeviceCmd& cmd);
  bool HandleNewProfVelCmd(DeviceCmd& cmd);
  bool HandleNewProfTorqueCmd(DeviceCmd& cmd);
  bool HandleNewHaltCmd();
  bool HandleNewResetCmd();
  bool HandleNewSetOutputPositionCmd(DeviceCmd& cmd);
  bool HandleNewSetUnitModeCmd(DeviceCmd& cmd);
  bool HandleNewCalibrationCmd(DeviceCmd& cmd);

  bool      IsIdleFaultConditionMet();
  bool      IsMotionFaultConditionMet();
  FaultType ProcessFaulted();
  FaultType ProcessHalted();
  FaultType ProcessHolding();
  FaultType ProcessProfPos();
  FaultType ProcessProfVel();
  FaultType ProcessProfTorque();
  FaultType ProcessCS();
  FaultType ProcessCalMoveToHardstop();
  FaultType ProcessCalAtHardstop();
  FaultType ProcessCalMoveToSoftstop();
  FaultType ProcessProfPosDisengaging();
  FaultType ProcessProfVelDisengaging();
  FaultType ProcessProfTorqueDisengaging();

  virtual void EgdRead();
  virtual void EgdSetConfig();
  virtual void EgdProcess();
  virtual void EgdClearErrors();
  virtual void EgdReset();
  virtual void EgdHalt();
  virtual void EgdSetPeakCurrent(double current);
  virtual void EgdSetUnitMode(int32_t mode, uint16_t app_id);
  virtual void EgdCSP(jsd_egd_motion_command_csp_t jsd_csp_cmd);
  virtual void EgdCSV(jsd_egd_motion_command_csv_t jsd_csv_cmd);
  virtual void EgdCST(jsd_egd_motion_command_cst_t jsd_cst_cmd);
  virtual void EgdSetGainSchedulingMode(jsd_egd_gain_scheduling_mode_t mode, uint16_t app_id);
  virtual void EgdSetGainSchedulingIndex(uint16_t index);

  std::string  actuator_type_str_;
  ActuatorType actuator_type_;

  double gear_ratio_                    = 1;
  double counts_per_rev_                = 1;
  double max_speed_eu_per_sec_          = 0;
  double max_accel_eu_per_sec2_         = 0;
  double over_speed_multiplier_         = 1;
  double vel_tracking_error_eu_per_sec_ = 0;
  double pos_tracking_error_eu_         = 0;
  double peak_current_limit_amps_       = 0;
  double peak_current_time_sec_         = 0;
  double continuous_current_limit_amps_ = 0;
  double torque_slope_amps_per_sec_     = 0;
  double low_pos_cal_limit_eu_          = 0;
  double low_pos_cmd_limit_eu_          = 0;
  double high_pos_cmd_limit_eu_         = 0;
  double high_pos_cal_limit_eu_         = 0;
  double holding_duration_sec_          = 0;
  double egd_brake_engage_msec_         = 0;
  double egd_brake_disengage_msec_      = 0;
  double egd_crc_                       = 0;
  double egd_drive_max_cur_limit_amps_  = 0;
  double smooth_factor_                 = 0;
  double torque_constant_               = 0;
  double winding_resistance_            = 0;
  double brake_power_                   = 0;
  double motor_encoder_gear_ratio_      = 0;

  bool compute_power_ = false;

  jsd_slave_config_t jsd_slave_config_;
  jsd_egd_state_t    jsd_egd_state_;
  DeviceCmd          last_cmd_;

  ActuatorStateMachineState actuator_sms_;
  double                    last_transition_time_;
  double                    last_egd_reset_time_;
  trap_t                    trap_;
  double                    overall_reduction_   = 1;
  int32_t                   egd_pos_offset_cnts_ = 1;

  ActuatorCalibrateCmd cal_cmd_;

 private:
  bool GSModeFromString(std::string                     gs_mode_string,
                        jsd_egd_gain_scheduling_mode_t& gs_mode);

  bool prof_pos_hold_;

  bool actuator_absolute_encoder_ = false;
};

}  // namespace fastcat

#endif
