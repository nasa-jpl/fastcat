#ifndef FASTCAT_ACTUATOR_H_
#define FASTCAT_ACTUATOR_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "fastcat/trap.h"
#include "jsd/jsd_elmo_common_types.h"

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

typedef enum {
  ACTUATOR_FASTCAT_FAULT_OKAY = 0,
  // Faults that can occur in handling of new commands
  ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED,
  ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION,
  ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL,
  ACTUATOR_FASTCAT_FAULT_INVALID_CAL_MOTION_RANGE,
  // Faults that can occur in processing of Actuator's state machine
  ACTUATOR_FASTCAT_FAULT_STO_ENGAGED,
  ACTUATOR_FASTCAT_FAULT_INVALID_ELMO_SMS_DURING_MOTION,
  ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED,
  ACTUATOR_FASTCAT_FAULT_NO_HARDSTOP_DURING_CAL,
  ACTUATOR_FASTCAT_FAULT_CAL_RESET_TIMEOUT_EXCEEDED,
  ACTUATOR_FASTCAT_FAULT_PROF_POS_CMD_ACK_TIMEOUT_EXCEEDED,
} ActuatorFastcatFault;

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

  static std::string GetFastcatFaultCodeAsString(const DeviceState& state);
  static std::string GetJSDFaultCodeAsString(const DeviceState& state);
  static bool        IsJsdFaultCodePresent(const DeviceState& state);
  static double      GetActualPosition(const DeviceState& state);

  struct ActuatorParams {
    std::string actuator_type_str;
    // TODO: populate ctrl_gs_mode_str from ctrl_gs_mode_ once JSD function is
    // available.
    std::string ctrl_gs_mode_str;
    double      gear_ratio                    = 1.0;
    double      counts_per_rev                = 1.0;
    double      max_speed_eu_per_sec          = 0.0;
    double      max_accel_eu_per_sec2         = 0.0;
    double      over_speed_multiplier         = 1.0;
    double      vel_tracking_error_eu_per_sec = 0.0;
    double      pos_tracking_error_eu         = 0.0;
    double      peak_current_limit_amps       = 0.0;
    double      peak_current_time_sec         = 0.0;
    double      continuous_current_limit_amps = 0.0;
    double      torque_slope_amps_per_sec     = 0.0;
    double      low_pos_cal_limit_eu          = 0.0;
    double      low_pos_cmd_limit_eu          = 0.0;
    double      high_pos_cmd_limit_eu         = 0.0;
    double      high_pos_cal_limit_eu         = 0.0;
    double      holding_duration_sec          = 0.0;
    double      elmo_brake_engage_msec        = 0.0;
    double      elmo_brake_disengage_msec     = 0.0;
    int64_t     elmo_crc                      = 0;
    double      elmo_drive_max_cur_limit_amps = 0.0;
    double      smooth_factor                 = 0.0;
    double      torque_constant               = 0.0;
    double      winding_resistance            = 0.0;
    double      brake_power                   = 0.0;
    double      motor_encoder_gear_ratio      = 0.0;
    bool        actuator_absolute_encoder     = false;
    bool        prof_pos_hold                 = false;
  };

  const ActuatorParams& GetParams() { return params_; }

 protected:

  double  CntsToEu(int32_t cnts);
  double  EuToCnts(double eu);
  double  PosCntsToEu(int32_t cnts);
  int32_t PosEuToCnts(double eu);

  void               TransitionToState(ActuatorStateMachineState sms);
  static std::string StateMachineStateToString(ActuatorStateMachineState sms);

  bool      IsMotionFaultConditionMet();
  FaultType ProcessProfPosTrapImpl();

  double ComputeTargetPosProfPosCmd(const DeviceCmd& cmd);
  double ComputePower(double actual_velocity, double actual_current,
                      bool motor_is_on);

  bool   compute_power_ = false;


  // Use mode saved in driver's volatile memory.
  jsd_elmo_gain_scheduling_mode_t ctrl_gs_mode_ =
      JSD_ELMO_GAIN_SCHEDULING_MODE_PRELOADED;

  jsd_slave_config_t jsd_slave_config_;

  ActuatorStateMachineState actuator_sms_;
  double                    last_transition_time_ = 0.0;
  double                    cycle_mono_time_      = 0.0;
  fastcat_trap_t            trap_;
  ActuatorParams            params_;

  ActuatorFastcatFault fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_OKAY;

  DeviceCmd last_cmd_;

 private:
  bool PosExceedsCmdLimits(double pos_eu);
  bool VelExceedsCmdLimits(double vel_eu);
  bool AccExceedsCmdLimits(double vel_eu);
  bool CurrentExceedsCmdLimits(double current);
  bool CheckStateMachineMotionCmds();
  bool CheckStateMachineGainSchedulingCmds();
  bool HandleNewHaltCmd();
  bool HandleNewResetCmd();
  bool HandleNewSetOutputPositionCmd(const DeviceCmd& cmd);
  bool HandleNewSetUnitModeCmd(const DeviceCmd& cmd);
  bool HandleNewCalibrationCmd(const DeviceCmd& cmd);
  bool HandleNewCSPCmd(const DeviceCmd& cmd);
  bool HandleNewCSVCmd(const DeviceCmd& cmd);
  bool HandleNewCSTCmd(const DeviceCmd& cmd);
  bool HandleNewProfPosCmd(const DeviceCmd& cmd);
  bool HandleNewProfVelCmd(const DeviceCmd& cmd);
  bool HandleNewProfTorqueCmd(const DeviceCmd& cmd);

  bool      IsIdleFaultConditionMet();
  FaultType ProcessFaulted();
  FaultType ProcessHalted();
  FaultType ProcessHolding();
  FaultType ProcessCS();
  FaultType ProcessCalMoveToHardstop();
  FaultType ProcessCalAtHardstop();
  FaultType ProcessCalMoveToSoftstop();

  bool GSModeFromString(std::string                      gs_mode_string,
                        jsd_elmo_gain_scheduling_mode_t& gs_mode);

  virtual bool ParseSpecializedYamlParams(const YAML::Node& node);

  virtual void PopulateJsdSlaveConfig() = 0;
  virtual void PopulateState()          = 0;

  virtual bool HandleNewProfPosCmdImpl(const DeviceCmd& cmd)    = 0;
  virtual bool HandleNewProfVelCmdImpl(const DeviceCmd& cmd)    = 0;
  virtual bool HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd) = 0;

  virtual FaultType ProcessProfPosDisengaging()    = 0;
  virtual FaultType ProcessProfPos()               = 0;
  virtual FaultType ProcessProfVelDisengaging()    = 0;
  virtual FaultType ProcessProfVel()               = 0;
  virtual FaultType ProcessProfTorqueDisengaging() = 0;
  virtual FaultType ProcessProfTorque()            = 0;

  virtual void ElmoSetConfig();
  virtual void ElmoRead() = 0;
  virtual void ElmoClearErrors() = 0;
  virtual void ElmoFault()                                                              = 0;
  virtual void ElmoReset()                                                              = 0;
  virtual void ElmoSetPeakCurrent(double current)                                       = 0;
  virtual void ElmoSetDigitalOutput(uint8_t digital_output_index, uint8_t output_level) = 0;
  virtual void ElmoSetUnitMode(int32_t mode, uint16_t app_id)                           = 0;
  virtual void ElmoSetGainSchedulingMode(jsd_elmo_gain_scheduling_mode_t mode,
                                         uint16_t app_id)                               = 0;
  virtual void ElmoSetGainSchedulingIndex(uint16_t index)                               = 0;
  virtual void ElmoCSP(const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)                = 0;
  virtual void ElmoCSV(const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)                = 0;
  virtual void ElmoCST(const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)                = 0;
  virtual void ElmoHalt()                                                               = 0;
  virtual void ElmoProcess()                                                            = 0;

  virtual double                         GetActualVelocity()        = 0;
  virtual double                         GetElmoActualPosition()    = 0;
  virtual jsd_elmo_state_machine_state_t GetElmoStateMachineState() = 0;
  virtual bool                           IsStoEngaged()             = 0;

  double overall_reduction_ = 1.0;

  ActuatorCalibrateCmd cal_cmd_;

  bool actuator_absolute_encoder_ = false;
  int32_t elmo_pos_offset_cnts_      = 1;

};

}  // namespace fastcat

#endif
