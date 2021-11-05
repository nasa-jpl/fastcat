// Include related header (for cc files)
#include "fastcat/jsd/actuator.h"

// Include c then c++ libraries
#include <string.h>
#include <sys/stat.h>

#include <cmath>
#include <fstream>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd.h"

fastcat::Actuator::Actuator()
{
  MSG_DEBUG("Constructed Actuator");

  state_                = std::make_shared<DeviceState>();
  state_->type          = ACTUATOR_STATE;
  actuator_sms_         = ACTUATOR_SMS_RESETTING;
  last_transition_time_ = jsd_get_time_sec();
  last_egd_reset_time_  = jsd_get_time_sec();
}

bool fastcat::Actuator::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }

  if (!ParseVal(node, "actuator_type", actuator_type_str_)) {
    return false;
  }

  if (0 == actuator_type_str_.compare("revolute")) {
    actuator_type_ = ACTUATOR_TYPE_REVOLUTE;

  } else if (0 == actuator_type_str_.compare("prismatic")) {
    actuator_type_ = ACTUATOR_TYPE_PRISMATIC;

  } else {
    ERROR("Failed to parse actuator_type string: %s must be %s or %s",
          actuator_type_str_.c_str(), "revolute", "prismatic");
    return false;
  }

  if (!ParseValCheckRange(node, "gear_ratio", gear_ratio_, 0, 1.0e12)) {
    return false;
  }

  if (!ParseValCheckRange(node, "counts_per_rev", counts_per_rev_, 0, 1.0e12)) {
    return false;
  }

  if (!ParseValCheckRange(node, "max_speed_eu_per_sec", max_speed_eu_per_sec_,
                          0, (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "max_accel_eu_per_sec2", max_accel_eu_per_sec2_,
                          0, (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "over_speed_multiplier", over_speed_multiplier_,
                          0.0, 1000.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "vel_tracking_error_eu_per_sec",
                          vel_tracking_error_eu_per_sec_, 0,
                          (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "pos_tracking_error_eu", pos_tracking_error_eu_,
                          0, (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "peak_current_limit_amps",
                          peak_current_limit_amps_, 0, 100)) {
    return false;
  }
  if (!ParseValCheckRange(node, "peak_current_time_sec", peak_current_time_sec_,
                          0, 60.0)) {
    return false;
  }
  if (!ParseValCheckRange(node, "continuous_current_limit_amps",
                          continuous_current_limit_amps_, 0, 100)) {
    return false;
  }
  if (!ParseValCheckRange(node, "torque_slope_amps_per_sec",
                          torque_slope_amps_per_sec_, 0, 1000.0)) {
    return false;
  }

  if (!ParseVal(node, "low_pos_cal_limit_eu", low_pos_cal_limit_eu_)) {
    return false;
  }
  if (!ParseVal(node, "low_pos_cmd_limit_eu", low_pos_cmd_limit_eu_)) {
    return false;
  }

  if (!ParseVal(node, "high_pos_cal_limit_eu", high_pos_cal_limit_eu_)) {
    return false;
  }
  if (!ParseVal(node, "high_pos_cmd_limit_eu", high_pos_cmd_limit_eu_)) {
    return false;
  }

  if (!ParseValCheckRange(node, "holding_duration_sec", holding_duration_sec_,
                          0, 1000.0)) {
    return false;
  }

  if (!ParseVal(node, "egd_brake_engage_msec", egd_brake_engage_msec_)) {
    return false;
  }
  if (!ParseVal(node, "egd_brake_disengage_msec", egd_brake_disengage_msec_)) {
    return false;
  }
  if (!ParseVal(node, "egd_crc", egd_crc_)) {
    return false;
  }
  if (!ParseVal(node, "egd_drive_max_current_limit",
                egd_drive_max_cur_limit_amps_)) {
    return false;
  }
  if (!ParseValCheckRange(node, "smooth_factor", smooth_factor_, -1, 64)) {
    return false;
  }

  // overall_reduction must be set before using EuToCnts/CntsToEu
  if (actuator_type_ == ACTUATOR_TYPE_REVOLUTE) {
    overall_reduction_ = counts_per_rev_ * gear_ratio_ / (2.0 * M_PI);

  } else if (actuator_type_ == ACTUATOR_TYPE_PRISMATIC) {
    overall_reduction_ = counts_per_rev_ * gear_ratio_;

  } else {
    ERROR("Bad actuator_type: %d", actuator_type_);
    return false;
  }
  MSG("Overall Reduction: %lf", overall_reduction_);

  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EGD_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  jsd_slave_config_.egd.drive_cmd_mode    = JSD_EGD_DRIVE_CMD_MODE_CS;
  jsd_slave_config_.egd.max_motor_speed   = EuToCnts(max_speed_eu_per_sec_);
  jsd_slave_config_.egd.loop_period_ms    = loop_period_ * 1000.0;
  jsd_slave_config_.egd.torque_slope      = torque_slope_amps_per_sec_;
  jsd_slave_config_.egd.max_profile_accel = EuToCnts(max_accel_eu_per_sec2_);
  jsd_slave_config_.egd.max_profile_decel = EuToCnts(max_accel_eu_per_sec2_);
  jsd_slave_config_.egd.velocity_tracking_error =
      EuToCnts(vel_tracking_error_eu_per_sec_);
  jsd_slave_config_.egd.position_tracking_error =
      EuToCnts(pos_tracking_error_eu_);
  jsd_slave_config_.egd.peak_current_limit = peak_current_limit_amps_;
  jsd_slave_config_.egd.peak_current_time  = peak_current_time_sec_;
  jsd_slave_config_.egd.continuous_current_limit =
      continuous_current_limit_amps_;
  jsd_slave_config_.egd.motor_stuck_current_level_pct  = 0;  // disable
  jsd_slave_config_.egd.motor_stuck_velocity_threshold = 0;  // disable
  jsd_slave_config_.egd.motor_stuck_timeout            = 0;  // disable
  jsd_slave_config_.egd.over_speed_threshold =
      over_speed_multiplier_ * EuToCnts(max_speed_eu_per_sec_);
  jsd_slave_config_.egd.low_position_limit      = 0;  // disable
  jsd_slave_config_.egd.high_position_limit     = 0;  // disable
  jsd_slave_config_.egd.brake_engage_msec       = egd_brake_engage_msec_;
  jsd_slave_config_.egd.brake_disengage_msec    = egd_brake_disengage_msec_;
  jsd_slave_config_.egd.crc                     = egd_crc_;
  jsd_slave_config_.egd.drive_max_current_limit = egd_drive_max_cur_limit_amps_;
  jsd_slave_config_.egd.smooth_factor           = smooth_factor_;

  EgdSetConfig();

  return true;
}

bool fastcat::Actuator::Read()
{
  EgdRead();

  state_->actuator_state.egd_actual_position = jsd_egd_state_.actual_position;
  state_->actuator_state.egd_cmd_position    = jsd_egd_state_.cmd_position;

  state_->actuator_state.actual_position =
      PosCntsToEu(jsd_egd_state_.actual_position);
  state_->actuator_state.actual_velocity =
      CntsToEu(jsd_egd_state_.actual_velocity);
  state_->actuator_state.actual_current = jsd_egd_state_.actual_current;

  state_->actuator_state.cmd_position =
      PosCntsToEu(jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position);
  state_->actuator_state.cmd_velocity =
      CntsToEu(jsd_egd_state_.cmd_velocity + jsd_egd_state_.cmd_ff_velocity);
  state_->actuator_state.cmd_current =
      (jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current);

  state_->actuator_state.egd_state_machine_state =
      jsd_egd_state_.actual_state_machine_state;
  state_->actuator_state.egd_mode_of_operation =
      jsd_egd_state_.actual_mode_of_operation;

  state_->actuator_state.sto_engaged       = jsd_egd_state_.sto_engaged;
  state_->actuator_state.hall_state        = jsd_egd_state_.hall_state;
  state_->actuator_state.target_reached    = jsd_egd_state_.target_reached;
  state_->actuator_state.motor_on          = jsd_egd_state_.motor_on;
  state_->actuator_state.fault_code        = jsd_egd_state_.fault_code;
  state_->actuator_state.bus_voltage       = jsd_egd_state_.bus_voltage;
  state_->actuator_state.drive_temperature = jsd_egd_state_.drive_temperature;
  state_->actuator_state.actuator_state_machine_state =
      static_cast<int>(actuator_sms_);

  return true;
}

bool fastcat::Actuator::Write(DeviceCmd& cmd)
{
  switch (cmd.type) {
    case ACTUATOR_CSP_CMD:
      if (!HandleNewCSPCmd(cmd)) {
        ERROR("Failed to handle Actuator CSP command");
        return false;
      }
      break;

    case ACTUATOR_CSV_CMD:
      if (!HandleNewCSVCmd(cmd)) {
        ERROR("Failed to handle Actuator CSV command");
        return false;
      }
      break;

    case ACTUATOR_CST_CMD:
      if (!HandleNewCSTCmd(cmd)) {
        ERROR("Failed to handle Actuator CST command");
        return false;
      }
      break;

    case ACTUATOR_PROF_POS_CMD:
      if (!HandleNewProfPosCmd(cmd)) {
        ERROR("Failed to setup Actuator Profiled Pos command");
        return false;
      }
      break;

    case ACTUATOR_PROF_VEL_CMD:
      if (!HandleNewProfVelCmd(cmd)) {
        ERROR("Failed to setup Actuator Profiled Pos command");
        return false;
      }
      break;

    case ACTUATOR_PROF_TORQUE_CMD:
      if (!HandleNewProfTorqueCmd(cmd)) {
        ERROR("Failed to setup Actuator Profiled Pos command");
        return false;
      }
      break;

    case ACTUATOR_RESET_CMD:
      Reset();
      break;

    case ACTUATOR_HALT_CMD:
      if (!HandleNewHaltCmd()) {
        ERROR("Failed to handle Halt Command");
        return false;
      }
      break;

    case ACTUATOR_SET_OUTPUT_POSITION_CMD:
      if (!HandleNewSetOutputPositionCmd(cmd)) {
        ERROR("Failed to handle Set Output Position Command");
        return false;
      }
      break;

    case ACTUATOR_CALIBRATE_CMD:
      if (!HandleNewCalibrationCmd(cmd)) {
        ERROR("Failed to handle Calibrate Command");
        return false;
      }
      break;

    default:
      WARNING("That command type is not supported in this mode!");
      return false;
  }

  return true;
}

fastcat::FaultType fastcat::Actuator::Process()
{
  fastcat::FaultType retval = NO_FAULT;

  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      break;

    case ACTUATOR_SMS_HALTED:
      retval = ProcessHalted();
      break;

    case ACTUATOR_SMS_HOLDING:
      retval = ProcessHolding();
      break;

    case ACTUATOR_SMS_PROF_POS:
      retval = ProcessProfPos();
      break;

    case ACTUATOR_SMS_PROF_VEL:
      retval = ProcessProfVel();
      break;

    case ACTUATOR_SMS_PROF_TORQUE:
      retval = ProcessProfTorque();
      break;

    case ACTUATOR_SMS_CS:
      retval = ProcessCS();
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
      retval = ProcessCalMoveToHardstop();
      break;

    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
      retval = ProcessCalAtHardstop();
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      retval = ProcessCalMoveToSoftstop();
      break;

    default:
      ERROR("Bad Actuator State Machine State: %d", actuator_sms_);
      retval = ALL_DEVICE_FAULT;
      break;
  }

  EgdProcess();
  return retval;
}

void fastcat::Actuator::Fault()
{
  WARNING("Faulting Actuator %s", name_.c_str());

  TransitionToState(ACTUATOR_SMS_FAULTED);
  EgdHalt();
}

void fastcat::Actuator::Reset()
{
  WARNING("Resetting Actuator device %s", name_.c_str());
  if (actuator_sms_ == ACTUATOR_SMS_FAULTED) {
    EgdReset();
    TransitionToState(ACTUATOR_SMS_RESETTING);
  }
}

bool fastcat::Actuator::SetOutputPosition(double position)
{
  MSG("Act %s: %s%lf %s%lf", name_.c_str(),
      "Changing Position from: ", state_->actuator_state.actual_position,
      "to : ", position);

  egd_pos_offset_cnts_ =
      jsd_egd_state_.actual_position - (int32_t)(position * overall_reduction_);
  return true;
}

double fastcat::Actuator::CntsToEu(int32_t cnts)
{
  return cnts / overall_reduction_;
}
int32_t fastcat::Actuator::EuToCnts(double eu)
{
  return eu * overall_reduction_;
}

double fastcat::Actuator::PosCntsToEu(int32_t cnts)
{
  return CntsToEu(cnts - egd_pos_offset_cnts_);
}
int32_t fastcat::Actuator::PosEuToCnts(double eu)
{
  return EuToCnts(eu) + egd_pos_offset_cnts_;
}

bool fastcat::Actuator::PosExceedsCmdLimits(double pos_eu)
{
  if (pos_eu > high_pos_cmd_limit_eu_) {
    ERROR("Commanded Position (%lf) exceeds high cmd limit (%lf)", pos_eu,
          high_pos_cmd_limit_eu_);
    return true;
  }
  if (pos_eu < low_pos_cmd_limit_eu_) {
    ERROR("Commanded Position (%lf) exceeds low cmd limit: (%lf)", pos_eu,
          low_pos_cmd_limit_eu_);
    return true;
  }
  return false;
}

bool fastcat::Actuator::VelExceedsCmdLimits(double vel_eu)
{
  if (fabs(vel_eu) > max_speed_eu_per_sec_) {
    ERROR("Commanded Velocity (%lf) exceeds max speed limit (%lf)", vel_eu,
          max_speed_eu_per_sec_);
    return true;
  }
  return false;
}

bool fastcat::Actuator::AccExceedsCmdLimits(double acc_eu)
{
  if (fabs(acc_eu) > max_accel_eu_per_sec2_) {
    ERROR("Commanded Acceleration (%lf) exceeds max limit (%lf)", acc_eu,
          max_accel_eu_per_sec2_);
    return true;
  }
  return false;
}

bool fastcat::Actuator::CurrentExceedsCmdLimits(double current)
{
  if (fabs(current) > continuous_current_limit_amps_) {
    WARNING(
        "Commanded current (%lf) exceeds max continuous current limit (%lf)",
        current, continuous_current_limit_amps_);
    // Not issuing fault for now
  }

  if (fabs(current) > peak_current_limit_amps_) {
    ERROR("Commanded current (%lf) exceeds max peak current limit (%lf)",
          current, peak_current_limit_amps_);
    return true;
  }

  return false;
}

void fastcat::Actuator::TransitionToState(ActuatorStateMachineState sms)
{
  if (actuator_sms_ == sms) {
    last_transition_time_ = state_->time;
    return;
  }

  MSG("Requested Actuator %s state transition from %s to %s", name_.c_str(),
      StateMachineStateToString(actuator_sms_).c_str(),
      StateMachineStateToString(sms).c_str());

  last_transition_time_ = state_->time;
  actuator_sms_         = sms;
}

std::string fastcat::Actuator::StateMachineStateToString(
    ActuatorStateMachineState sms)
{
  std::string str;
  switch (sms) {
    case ACTUATOR_SMS_FAULTED:
      str = std::string("FAULTED");
      break;
    case ACTUATOR_SMS_HALTED:
      str = std::string("HALTED");
      break;
    case ACTUATOR_SMS_HOLDING:
      str = std::string("HOLDING");
      break;
    case ACTUATOR_SMS_PROF_POS:
      str = std::string("PROF_POS");
      break;
    case ACTUATOR_SMS_PROF_VEL:
      str = std::string("PROF_VEL");
      break;
    case ACTUATOR_SMS_PROF_TORQUE:
      str = std::string("PROF_TORQUE");
      break;
    case ACTUATOR_SMS_CS:
      str = std::string("CS");
      break;
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
      str = std::string("CAL_MOVE_TO_HARDSTOP");
      break;
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
      str = std::string("CAL_AT_HARDSTOP");
      break;
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      str = std::string("CAL_MOVE_TO_SOFTSTOP");
      break;
    default:
      ERROR("Bad Actuator State Machine State: %d", static_cast<int>(sms));
  }
  return str;
}

void fastcat::Actuator::EgdRead()
{
  jsd_egd_read((jsd_t*)context_, slave_id_);
  memcpy(&jsd_egd_state_, jsd_egd_get_state((jsd_t*)context_, slave_id_),
         sizeof(jsd_egd_state_t));
}

void fastcat::Actuator::EgdSetConfig()
{
  MSG_DEBUG("Setting jsd slave config");
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
}
void fastcat::Actuator::EgdProcess()
{
  jsd_egd_process((jsd_t*)context_, slave_id_);
}

void fastcat::Actuator::EgdReset()
{
  if ((state_->time - last_egd_reset_time_) > 1.0) {
    MSG("Resetting EGD through JSD: %s", name_.c_str());
    jsd_egd_reset((jsd_t*)context_, slave_id_);
    last_egd_reset_time_ = state_->time;
  }
}

void fastcat::Actuator::EgdHalt() { jsd_egd_halt((jsd_t*)context_, slave_id_); }

void fastcat::Actuator::EgdSetPeakCurrent(double current)
{
  jsd_egd_set_peak_current((jsd_t*)context_, slave_id_, current);
}

void fastcat::Actuator::EgdCSP(jsd_egd_motion_command_csp_t jsd_csp_cmd)
{
  jsd_egd_set_motion_command_csp((jsd_t*)context_, slave_id_, jsd_csp_cmd);
}

void fastcat::Actuator::EgdCSV(jsd_egd_motion_command_csv_t jsd_csv_cmd)
{
  jsd_egd_set_motion_command_csv((jsd_t*)context_, slave_id_, jsd_csv_cmd);
}

void fastcat::Actuator::EgdCST(jsd_egd_motion_command_cst_t jsd_cst_cmd)
{
  jsd_egd_set_motion_command_cst((jsd_t*)context_, slave_id_, jsd_cst_cmd);
}
