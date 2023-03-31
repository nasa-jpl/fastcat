// Include related header (for cc files)
#include "fastcat/jsd/egd_actuator_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>

// Include external then project includes
#include "jsd/jsd.h"
#include "jsd/jsd_time.h"

fastcat::EgdActuatorOffline::EgdActuatorOffline()
{
  MSG_DEBUG("Constructed EgdActuatorOffline");

  memset(&jsd_egd_state_, 0, sizeof(jsd_egd_state_t));
  motor_on_start_time_ = jsd_time_get_time_sec();
}

void fastcat::EgdActuatorOffline::ElmoSetConfig()
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoRead()
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoProcess()
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
    case ACTUATOR_SMS_HALTED:
      jsd_egd_state_.motor_on      = 0;
      jsd_egd_state_.servo_enabled = 0;
      break;

    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CS:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
    default:
      jsd_egd_state_.motor_on = 1;
      break;
  }

  // reset motor_on timer on rising edge
  if (!last_motor_on_state_ && jsd_egd_state_.motor_on) {
    motor_on_start_time_ = jsd_time_get_time_sec();
  }
  last_motor_on_state_ = jsd_egd_state_.motor_on;

  //
  if (!jsd_egd_state_.servo_enabled && jsd_egd_state_.motor_on) {
    double brake_on_dur = jsd_time_get_time_sec() - motor_on_start_time_;
    if (brake_on_dur > elmo_brake_disengage_msec_ / 1000.0) {
      jsd_egd_state_.servo_enabled = 1;
    }
  }
}

void fastcat::EgdActuatorOffline::ElmoReset()
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoHalt()
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoClearErrors()
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoSetPeakCurrent(double /* current */)
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoSetUnitMode(int32_t mode, uint16_t app_id)
{
  MSG("Commanding new UM[1] = %d app_id = %u", mode, app_id);
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoSetGainSchedulingMode(
    jsd_elmo_gain_scheduling_mode_t /* mode */, uint16_t /* app_id */)
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoSetGainSchedulingIndex(
    uint16_t /* index */)
{
  // no-op
}

void fastcat::EgdActuatorOffline::ElmoCSP(
    const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)
{
  jsd_egd_state_.cmd_position = jsd_csp_cmd.target_position;
  jsd_egd_state_.cmd_velocity = 0;
  jsd_egd_state_.cmd_current  = 0;

  jsd_egd_state_.cmd_ff_position = jsd_csp_cmd.position_offset;
  jsd_egd_state_.cmd_ff_velocity = jsd_csp_cmd.velocity_offset;
  jsd_egd_state_.cmd_ff_current  = jsd_csp_cmd.torque_offset_amps;

  // Differentiate position to get actual_velocity
  double vel = (jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position -
                jsd_egd_state_.actual_position) /
               loop_period_;

  // simulate actuals
  jsd_egd_state_.actual_position =
      jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position;
  jsd_egd_state_.actual_current =
      jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current;

  jsd_egd_state_.actual_velocity = vel;  // "sure, why not"
}

void fastcat::EgdActuatorOffline::ElmoCSV(
    const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)
{
  jsd_egd_state_.cmd_position = 0;
  jsd_egd_state_.cmd_velocity = jsd_csv_cmd.target_velocity;
  jsd_egd_state_.cmd_current  = 0;

  jsd_egd_state_.cmd_ff_position = 0;
  jsd_egd_state_.cmd_ff_velocity = jsd_csv_cmd.velocity_offset;
  jsd_egd_state_.cmd_ff_current  = jsd_csv_cmd.torque_offset_amps;

  // simulate actuals
  jsd_egd_state_.actual_velocity =
      jsd_egd_state_.cmd_velocity + jsd_egd_state_.cmd_ff_velocity;
  jsd_egd_state_.actual_current =
      jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current;
  jsd_egd_state_.actual_position +=
      jsd_egd_state_.actual_velocity * loop_period_;  // integrated
}

void fastcat::EgdActuatorOffline::ElmoCST(
    const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)
{
  jsd_egd_state_.cmd_position = 0;
  jsd_egd_state_.cmd_velocity = 0;
  jsd_egd_state_.cmd_current  = jsd_cst_cmd.target_torque_amps;

  jsd_egd_state_.cmd_ff_position = 0;
  jsd_egd_state_.cmd_ff_velocity = 0;
  jsd_egd_state_.cmd_ff_current  = jsd_cst_cmd.torque_offset_amps;

  // simulate actuals
  jsd_egd_state_.actual_position =
      jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position;
  jsd_egd_state_.actual_current =
      jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current;

  double pct = jsd_egd_state_.actual_current / continuous_current_limit_amps_;
  jsd_egd_state_.actual_velocity =
      pct * max_speed_eu_per_sec_;  // sure, why not
  jsd_egd_state_.actual_position +=
      jsd_egd_state_.actual_velocity * loop_period_;  // integrated
}
