// Include related header (for cc files)
#include "fastcat/jsd/platinum_sil_actuator.h"

// Include C then C++ libraries
#include <cmath>
#include <cstring>

// Include external then project includes
#include "fastcat/jsd/actuator_utils.h"
#include "fastcat/types.h"
#include "fastcat/yaml_parser.h"

fastcat::PlatinumSilActuator::PlatinumSilActuator()
{
  MSG_DEBUG("Constructed Platinum-SIL Actuator");

  state_        = std::make_shared<DeviceState>();
  state_->type  = PLATINUM_SIL_ACTUATOR_STATE;
  actuator_sms_ = PLATINUM_SIL_ACTUATOR_SMS_HALTED;
}

bool fastcat::PlatinumSilActuator::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config(context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::PlatinumSilActuator::ConfigFromYamlCommon(YAML::Node node)
{
  // TODO(dloret): Set SIL-related parameters
  if (!ParseVal(node, "name", name_)) {
    return false;
  }

  double gear_ratio = 1.0;
  if (!ParseValCheckRange(node, "gear_ratio", gear_ratio, 0.0, 1.0e12)) {
    return false;
  }

  double counts_per_rev = 1.0;
  if (!ParseValCheckRange(node, "counts_per_rev", counts_per_rev, 0.0,
                          1.0e12)) {
    return false;
  }

  std::string actuator_type_str;
  if (!ParseVal(node, "actuator_type", actuator_type_str)) {
    return false;
  }
  if (0 == actuator_type_str.compare("revolute")) {
    overall_reduction_ = counts_per_rev * gear_ratio / (2.0 * M_PI);
  } else if (0 == actuator_type_str.compare("prismatic")) {
    overall_reduction_ = counts_per_rev * gear_ratio;
  } else {
    ERROR("Failed to parse actuator_type string: %s must be %s or %s",
          actuator_type_str.c_str(), "revolute", "prismatic");
    return false;
  }
  MSG("Overall Reduction: %lf", overall_reduction_);

  double max_speed_eu_per_sec = 0.0;
  if (!ParseValCheckRange(node, "max_speed_eu_per_sec", max_speed_eu_per_sec,
                          0.0, CntsToEu(100000000000))) {
    return false;
  }

  float peak_current_limit_amps = 0.0;
  if (!ParseValCheckRange(node, "peak_current_limit_amps",
                          peak_current_limit_amps, 0.0f, 100.0f)) {
    return false;
  }

  float peak_current_time_sec = 0.0;
  if (!ParseValCheckRange(node, "peak_current_time_sec", peak_current_time_sec,
                          0.0f, 60.0f)) {
    return false;
  }

  float continuous_current_limit_amps = 0.0;
  if (!ParseValCheckRange(node, "continuous_current_limit_amps",
                          continuous_current_limit_amps, 0.0f, 100.0f)) {
    return false;
  }

  int16_t elmo_brake_engage_msec = 0;
  if (!ParseVal(node, "elmo_brake_engage_msec", elmo_brake_engage_msec)) {
    return false;
  }

  int16_t elmo_brake_disengage_msec = 0;
  if (!ParseVal(node, "elmo_brake_disengage_msec", elmo_brake_disengage_msec)) {
    return false;
  }

  uint32_t elmo_crc = 0;
  if (!ParseVal(node, "elmo_crc", elmo_crc)) {
    return false;
  }

  bool actuator_absolute_encoder = false;
  ParseOptVal(node, "absolute_encoder", actuator_absolute_encoder);

  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());
  jsd_slave_config_.driver_type                = JSD_DRIVER_TYPE_EPD_SIL;
  jsd_slave_config_.epd_sil.peak_current_limit = peak_current_limit_amps;
  jsd_slave_config_.epd_sil.peak_current_time  = peak_current_time_sec;
  jsd_slave_config_.epd_sil.continuous_current_limit =
      continuous_current_limit_amps;
  jsd_slave_config_.epd_sil.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection
  jsd_slave_config_.epd_sil.motor_stuck_velocity_threshold =
      0.0f;  // Motor stuck protection is disabled;
  jsd_slave_config_.epd_sil.motor_stuck_timeout =
      0.0f;  // Motor stuck protection is disabled
  jsd_slave_config_.epd_sil.over_speed_threshold =
      EuToCnts(max_speed_eu_per_sec);
  jsd_slave_config_.epd_sil.low_position_limit =
      0.0;  // Disable out of position limits protection
  jsd_slave_config_.epd_sil.high_position_limit =
      0.0;  // Disable out of position limits protection
  jsd_slave_config_.epd_sil.brake_engage_msec    = elmo_brake_engage_msec;
  jsd_slave_config_.epd_sil.brake_disengage_msec = elmo_brake_disengage_msec;
  jsd_slave_config_.epd_sil.crc                  = elmo_crc;

  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);

  return true;
}

bool fastcat::PlatinumSilActuator::Read()
{
  // TODO(dloret): Read SIL R1/R2 variables
  jsd_epd_sil_read(context_, slave_id_);
  memcpy(&jsd_epd_sil_state_, jsd_epd_sil_get_state(context_, slave_id_),
         sizeof(jsd_epd_sil_state_t));

  state_->platinum_sil_actuator_state.actual_position =
      PosCntsToEu(jsd_epd_sil_state_.actual_position);
  state_->platinum_sil_actuator_state.actual_velocity =
      CntsToEu(jsd_epd_sil_state_.actual_velocity);
  state_->platinum_sil_actuator_state.actual_current =
      jsd_epd_sil_state_.actual_current;

  state_->platinum_sil_actuator_state.cmd_max_current =
      jsd_epd_sil_state_.cmd_max_current;

  state_->platinum_sil_actuator_state.elmo_state_machine_state =
      static_cast<uint32_t>(jsd_epd_sil_state_.actual_state_machine_state);
  state_->platinum_sil_actuator_state.elmo_mode_of_operation =
      static_cast<uint32_t>(jsd_epd_sil_state_.actual_mode_of_operation);

  state_->platinum_sil_actuator_state.sto_engaged =
      jsd_epd_sil_state_.sto_engaged;
  state_->platinum_sil_actuator_state.hall_state =
      jsd_epd_sil_state_.hall_state;
  state_->platinum_sil_actuator_state.target_reached =
      jsd_epd_sil_state_.target_reached;
  state_->platinum_sil_actuator_state.motor_on = jsd_epd_sil_state_.motor_on;
  state_->platinum_sil_actuator_state.servo_enabled =
      jsd_epd_sil_state_.servo_enabled;

  state_->platinum_sil_actuator_state.jsd_fault_code =
      static_cast<uint32_t>(jsd_epd_sil_state_.fault_code);
  state_->platinum_sil_actuator_state.fastcat_fault_code =
      static_cast<uint32_t>(fastcat_fault_);
  state_->platinum_sil_actuator_state.emcy_error_code =
      jsd_epd_sil_state_.emcy_error_code;
  state_->platinum_sil_actuator_state.faulted =
      (actuator_sms_ == PLATINUM_SIL_ACTUATOR_SMS_FAULTED);

  state_->platinum_sil_actuator_state.elmo_actual_position =
      jsd_epd_sil_state_.actual_position;

  state_->platinum_sil_actuator_state.sil_initialized =
      jsd_epd_sil_state_.sil_initialized;
  state_->platinum_sil_actuator_state.sil_running =
      jsd_epd_sil_state_.sil_running;
  state_->platinum_sil_actuator_state.sil_faulted =
      jsd_epd_sil_state_.sil_faulted;

  return true;
}

fastcat::FaultType fastcat::PlatinumSilActuator::Process()
{
  FaultType fault = NO_FAULT;
  switch (actuator_sms_) {
    case PLATINUM_SIL_ACTUATOR_SMS_FAULTED:
      fault = ProcessFaulted();
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_HALTED:
      fault = ProcessHalted();
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_MOTION:
      fault = ProcessMotion();
      break;
    default:
      ERROR("Bad Actuator State Machine State: %d", actuator_sms_);
      fault = ALL_DEVICE_FAULT;
  }
  jsd_epd_sil_process(context_, slave_id_);
  return fault;
}

bool fastcat::PlatinumSilActuator::Write(DeviceCmd& cmd)
{
  // TODO(dloret): add handling of SIL-related commands
  bool success = false;
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    success = (sdoResult == SDO_RET_VAL_SUCCESS);
    // Handle non-motion commands always
  } else if (cmd.type == ACTUATOR_RESET_CMD) {
    Reset();
    success = true;
  } else if (cmd.type == ACTUATOR_SET_OUTPUT_POSITION_CMD) {
    if (!HandleNewSetOutputPositionCmd(cmd)) {
      ERROR("Failed to handle Set Output Position Command");
      success = false;
    } else {
      success = true;
    }
  } else if (cmd.type == ACTUATOR_SET_MAX_CURRENT_CMD) {
    jsd_epd_sil_set_peak_current(context_, slave_id_,
                                 cmd.actuator_set_max_current_cmd.current);
    success = true;
  } else if (cmd.type == ACTUATOR_SDO_SET_UNIT_MODE_CMD) {
    if (!HandleNewSetUnitModeCmd(cmd)) {
      ERROR("Failed to handle Set Unit Mode Command");
      success = false;
    } else {
      success = true;
    }
    // Ignore motion-based commands if a fault is active
  } else if (device_fault_active_) {
    success = false;
  } else if (cmd.type == ACTUATOR_HALT_CMD) {
    if (!HandleNewHaltCmd()) {
      ERROR("Failed to handle Halt Command");
      success = false;
    } else {
      success = true;
    }
  } else {
    WARNING("That command type is not supported!");
    success = false;
  }

  return success;
}

void fastcat::PlatinumSilActuator::Fault()
{
  // TODO(dloret): confirm why actuator drivers do not call DeviceBase::Fault()
  WARNING("Faulting Platinum-SIL Actuator %s", name_.c_str());
  DeviceBase::Fault();

  TransitionToState(PLATINUM_SIL_ACTUATOR_SMS_FAULTED);
  jsd_epd_sil_halt((jsd_t*)context_, slave_id_);
}

void fastcat::PlatinumSilActuator::Reset()
{
  WARNING("Resetting Platinum-SIL Actuator device %s", name_.c_str());
  if (actuator_sms_ == PLATINUM_SIL_ACTUATOR_SMS_FAULTED) {
    // Resetting here would open brakes so we explicitly do not reset the Elmo
    // drive and instead only clear latched errors
    jsd_epd_sil_clear_errors(context_, slave_id_);
    fastcat_fault_ = PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_OKAY;
    TransitionToState(PLATINUM_SIL_ACTUATOR_SMS_HALTED);
  }
}

double fastcat::PlatinumSilActuator::GetActualPosition()
{
  return state_->platinum_sil_actuator_state.actual_position;
}

bool fastcat::PlatinumSilActuator::SetOutputPosition(double position)
{
  MSG("Platinum-SIL actuator %s: %s%lf %s%lf", name_.c_str(),
      "Changing Position from: ",
      state_->platinum_sil_actuator_state.actual_position, "to : ", position);

  elmo_pos_offset_cnts_ =
      state_->platinum_sil_actuator_state.elmo_actual_position -
      (int32_t)(position * overall_reduction_);

  return true;
}

bool fastcat::PlatinumSilActuator::HasAbsoluteEncoder()
{
  return actuator_absolute_encoder_;
}

std::string fastcat::PlatinumSilActuator::FastcatFaultToString(
    PlatinumSilActuatorFastcatFault fault)
{
  std::string fault_str;

  switch (fault) {
    case PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_OKAY:
      fault_str = "FASTCAT_FAULT_OKAY";
      break;
    case PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION:
      fault_str = "FASTCAT_FAULT_INVALID_CMD_DURING_MOTION";
      break;
    case PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_STO_ENGAGED:
      fault_str = "FASTCAT_FAULT_STO_ENGAGED";
      break;
    case PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_ELMO_SMS_DURING_MOTION:
      fault_str = "FASTCAT_FAULT_INVALID_ELMO_SMS_DURING_MOTION";
      break;
    default:
      fault_str = "Bad Fastcat fault code";
  }

  return fault_str;
}

void fastcat::PlatinumSilActuator::TransitionToState(
    PlatinumSilActuatorStateMachineState sms)
{
  if (actuator_sms_ != sms) {
    MSG("Requested Actuator %s state transition from %s to %s", name_.c_str(),
        StateMachineStateToString(actuator_sms_).c_str(),
        StateMachineStateToString(sms).c_str());
    actuator_sms_ = sms;
  }
}

std::string fastcat::PlatinumSilActuator::StateMachineStateToString(
    PlatinumSilActuatorStateMachineState sms)
{
  std::string str;
  switch (sms) {
    case PLATINUM_SIL_ACTUATOR_SMS_FAULTED:
      str = std::string("FAULTED");
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_HALTED:
      str = std::string("HALTED");
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_MOTION:
      str = std::string("MOTION");
      break;
    default:
      ERROR("Bad Platinum-SIL Actuator State Machine State: %d",
            static_cast<int>(sms));
  }
  return str;
}

double fastcat::PlatinumSilActuator::CntsToEu(int64_t cnts)
{
  return cnts / overall_reduction_;
}

double fastcat::PlatinumSilActuator::EuToCnts(double eu)
{
  return eu * overall_reduction_;
}

double fastcat::PlatinumSilActuator::PosCntsToEu(int64_t cnts)
{
  return CntsToEu(cnts - elmo_pos_offset_cnts_);
}

int32_t fastcat::PlatinumSilActuator::PosEuToCnts(double eu)
{
  return ((int32_t)EuToCnts(eu)) + elmo_pos_offset_cnts_;
}

fastcat::FaultType fastcat::PlatinumSilActuator::ProcessFaulted()
{
  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumSilActuator::ProcessHalted()
{
  FaultType fault = NO_FAULT;
  if (IsIdleFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault condition present, faulting");
    fault = ALL_DEVICE_FAULT;
  }
  return fault;
}

fastcat::FaultType fastcat::PlatinumSilActuator::ProcessMotion()
{
  FaultType fault = NO_FAULT;
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault condition present, faulting");
    fault = ALL_DEVICE_FAULT;
  }
  return fault;
}

bool fastcat::PlatinumSilActuator::HandleNewHaltCmd()
{
  bool success = false;
  switch (actuator_sms_) {
    case PLATINUM_SIL_ACTUATOR_SMS_FAULTED:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot HALT from FAULTED state, reset actuator first");
      success = false;
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_HALTED:
      success = true;
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_MOTION:
      jsd_epd_sil_halt(context_, slave_id_);
      TransitionToState(PLATINUM_SIL_ACTUATOR_SMS_HALTED);
      success = true;
      break;
    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      success = false;
  }

  return success;
}

bool fastcat::PlatinumSilActuator::HandleNewSetOutputPositionCmd(
    const DeviceCmd& cmd)
{
  bool success = false;
  switch (actuator_sms_) {
    case PLATINUM_SIL_ACTUATOR_SMS_FAULTED:
    case PLATINUM_SIL_ACTUATOR_SMS_HALTED:
      SetOutputPosition(cmd.actuator_set_output_position_cmd.position);
      success = true;
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_MOTION:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot set output position, motion command is active");
      fastcat_fault_ =
          PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      success = false;
      break;
    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      success = false;
  }

  return success;
}

bool fastcat::PlatinumSilActuator::HandleNewSetUnitModeCmd(const DeviceCmd& cmd)
{
  bool success = false;
  switch (actuator_sms_) {
    case PLATINUM_SIL_ACTUATOR_SMS_FAULTED:
    case PLATINUM_SIL_ACTUATOR_SMS_HALTED:
      jsd_epd_sil_async_sdo_set_unit_mode(
          context_, slave_id_,
          static_cast<int16_t>(cmd.actuator_sdo_set_unit_mode_cmd.mode),
          cmd.actuator_sdo_set_unit_mode_cmd.app_id);
      success = true;
      break;
    case PLATINUM_SIL_ACTUATOR_SMS_MOTION:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot set unit mode now, motion command is active");
      fastcat_fault_ =
          PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      success = false;
      break;
    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      success = false;
  }

  return success;
}

bool fastcat::PlatinumSilActuator::IsIdleFaultConditionMet()
{
  bool condition_is_met = false;
  if (state_->platinum_sil_actuator_state.sto_engaged) {
    ERROR("%s: STO Engaged", name_.c_str());
    fastcat_fault_   = PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;
    condition_is_met = true;
  } else if (actuator_utils::IsJsdFaultCodePresent(*state_)) {
    ERROR("%s: jsd_fault_code indicates active fault", name_.c_str());
    condition_is_met = true;
  }
  return condition_is_met;
}

bool fastcat::PlatinumSilActuator::IsMotionFaultConditionMet()
{
  bool condition_is_met = false;
  if (state_->platinum_sil_actuator_state.sto_engaged) {
    ERROR("%s: STO Engaged", name_.c_str());
    fastcat_fault_   = PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;
    condition_is_met = true;
  } else if (actuator_utils::IsJsdFaultCodePresent(*state_)) {
    ERROR("%s: jsd_fault_code indicates active fault", name_.c_str());
    condition_is_met = true;
  } else if (GetElmoStateMachineState() ==
                 JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE ||
             GetElmoStateMachineState() ==
                 JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE ||
             GetElmoStateMachineState() == JSD_ELMO_STATE_MACHINE_STATE_FAULT) {
    ERROR("%s: Elmo drive state machine state is off nominal", name_.c_str());
    fastcat_fault_ =
        PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_ELMO_SMS_DURING_MOTION;
    condition_is_met = true;
  }
  return condition_is_met;
}