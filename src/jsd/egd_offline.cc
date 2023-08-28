// Include related header (for cc files)
#include "fastcat/jsd/egd_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes

fastcat::EgdOffline::EgdOffline() { MSG_DEBUG("Constructed EgdOffline"); }

bool fastcat::EgdOffline::ConfigFromYaml(const YAML::Node& node,
                                         double /*external_time*/)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::EgdOffline::Read()
{
  state_->egd_state.faulted = true;

  switch (jsd_slave_config_.egd.drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_PROFILED:
      if (!ReadProfiledMode()) {
        ERROR("Bad EgdOffline::ReadProfiledMode");
        return false;
      }
      break;

    case JSD_EGD_DRIVE_CMD_MODE_CS:
      if (!ReadCSMode()) {
        ERROR("Bad EgdOffline::ReadCSMode");
        return false;
      }
      break;

    default:
      ERROR("Bad Drive cmd mode");
  }

  state_->egd_state.actual_position = jsd_egd_state_.actual_position;
  state_->egd_state.actual_velocity = jsd_egd_state_.actual_velocity;
  state_->egd_state.actual_current  = jsd_egd_state_.actual_current;
  state_->egd_state.cmd_position    = jsd_egd_state_.cmd_position;
  state_->egd_state.cmd_velocity    = jsd_egd_state_.cmd_velocity;
  state_->egd_state.cmd_current     = jsd_egd_state_.cmd_current;
  state_->egd_state.target_reached  = jsd_egd_state_.target_reached;
  state_->egd_state.faulted         = false;

  return true;
}

fastcat::FaultType fastcat::EgdOffline::Process()
{
  return DeviceBase::Process();
}

bool fastcat::EgdOffline::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  switch (jsd_slave_config_.egd.drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_PROFILED:
      return WriteProfiledMode(cmd);
      break;

    case JSD_EGD_DRIVE_CMD_MODE_CS:
      return WriteCSMode(cmd);
      break;

    default:
      ERROR("Bad Drive cmd mode");
  }
  return false;
}

void fastcat::EgdOffline::Fault()
{
  DeviceBase::Fault();
  jsd_egd_state_.target_reached           = true;
  jsd_egd_state_.actual_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
}

void fastcat::EgdOffline::Reset() { DeviceBase::Reset(); }

bool fastcat::EgdOffline::ReadProfiledMode()
{
  double error = 0;
  double delta = 0;

  // Inside this switch, only set the jsd_egd_state_.actual* fields
  switch (jsd_egd_state_.actual_mode_of_operation) {
    case JSD_EGD_MODE_OF_OPERATION_DISABLED:
      jsd_egd_state_.actual_velocity = 0;
      jsd_egd_state_.actual_current  = 0;
      break;

    case JSD_EGD_MODE_OF_OPERATION_PROF_POS:
      // To simplify this profiling, we will advance position at
      // the profile_velocity with instantaneous accel/decel, ignoring
      // the profile_accel and profile_decel parameters

      // velocity
      if (jsd_egd_state_.target_reached) {
        // if the target_position has been reached, continue on at end_velocity
        jsd_egd_state_.actual_velocity = jsd_motion_cmd_.prof_pos.end_velocity;
        jsd_egd_state_.cmd_velocity    = jsd_motion_cmd_.prof_pos.end_velocity;
      } else {
        jsd_egd_state_.actual_velocity =
            jsd_motion_cmd_.prof_pos.profile_velocity;
        jsd_egd_state_.cmd_velocity = jsd_motion_cmd_.prof_pos.profile_velocity;
      }

      // position
      if (!jsd_egd_state_.target_reached) {
        error = jsd_motion_cmd_.prof_pos.target_position -
                jsd_egd_state_.actual_position;
        delta = copysign(jsd_egd_state_.actual_velocity * loop_period_, error);
        if (abs(error) < abs(delta)) {
          jsd_egd_state_.actual_position =
              jsd_motion_cmd_.prof_pos.target_position;
          jsd_egd_state_.target_reached = true;
        } else {
          jsd_egd_state_.actual_position += delta;
        }
      } else {
        // continue at at end_velocity rate, target_position has been reached
        delta = copysign(jsd_egd_state_.actual_velocity * loop_period_, error);
        jsd_egd_state_.actual_position += delta;
      }

      // current
      if (jsd_egd_state_.actual_velocity != 0) {
        jsd_egd_state_.actual_current =
            jsd_slave_config_.egd.continuous_current_limit;  // why not :)
      } else {
        jsd_egd_state_.actual_current = 0;
      }

      break;

    case JSD_EGD_MODE_OF_OPERATION_PROF_VEL:

      // velocity
      error = jsd_motion_cmd_.prof_vel.target_velocity -
              jsd_egd_state_.actual_velocity;
      delta = copysign(jsd_motion_cmd_.prof_vel.profile_accel * loop_period_,
                       error);
      if (abs(error) < abs(delta)) {
        jsd_egd_state_.actual_velocity = jsd_egd_state_.cmd_velocity;
        jsd_egd_state_.target_reached  = true;
      } else {
        jsd_egd_state_.actual_velocity += delta;
      }

      // position
      jsd_egd_state_.actual_position +=
          jsd_egd_state_.actual_velocity * loop_period_;

      // current
      if (jsd_egd_state_.actual_velocity != 0) {
        jsd_egd_state_.actual_current =
            jsd_slave_config_.egd.continuous_current_limit;  // why not :)
      } else {
        jsd_egd_state_.actual_current = 0;
      }
      break;

    case JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE:

      // velocity
      if (jsd_motion_cmd_.prof_torque.target_torque_amps < 1e-6) {
        jsd_egd_state_.actual_velocity = 0;

      } else {
        error = jsd_slave_config_.egd.max_motor_speed -
                abs(jsd_egd_state_.actual_velocity);
        delta = copysign(jsd_slave_config_.egd.max_profile_accel * loop_period_,
                         jsd_motion_cmd_.prof_torque.target_torque_amps);
        if (error < abs(delta)) {
          jsd_egd_state_.actual_velocity =
              jsd_slave_config_.egd.max_motor_speed;
        } else {
          jsd_egd_state_.actual_velocity += delta;
        }
      }

      // position
      jsd_egd_state_.actual_position +=
          jsd_egd_state_.actual_velocity * loop_period_;

      // current
      jsd_egd_state_.actual_current =
          jsd_motion_cmd_.prof_torque.target_torque_amps;
      break;

    default:
      ERROR("only CSP, CSV, and CST mode of operations supported");
      return false;
  }
  return true;
}

bool fastcat::EgdOffline::ReadCSMode()
{
  double delta = 0;
  // Inside this switch, only set the jsd_egd_state_.actual* fields
  switch (jsd_egd_state_.actual_mode_of_operation) {
    case JSD_EGD_MODE_OF_OPERATION_DISABLED:
      jsd_egd_state_.actual_velocity = 0;
      jsd_egd_state_.actual_current  = 0;
      break;

    case JSD_EGD_MODE_OF_OPERATION_CSP:
      delta = (jsd_motion_cmd_.csp.target_position +
               jsd_motion_cmd_.csp.position_offset) -
              jsd_egd_state_.actual_position;
      jsd_egd_state_.actual_position = (jsd_motion_cmd_.csp.target_position +
                                        jsd_motion_cmd_.csp.position_offset);
      jsd_egd_state_.actual_velocity = delta * loop_period_;
      if (jsd_egd_state_.actual_velocity != 0) {
        jsd_egd_state_.actual_current =
            jsd_slave_config_.egd.continuous_current_limit;  // why not :)
      } else {
        jsd_egd_state_.actual_current = 0;
      }
      break;

    case JSD_EGD_MODE_OF_OPERATION_CSV:
      jsd_egd_state_.actual_velocity = (jsd_motion_cmd_.csv.target_velocity +
                                        jsd_motion_cmd_.csv.velocity_offset);
      jsd_egd_state_.actual_position +=
          jsd_egd_state_.actual_velocity * loop_period_;

      if (jsd_egd_state_.actual_velocity != 0) {
        jsd_egd_state_.actual_current =
            jsd_slave_config_.egd.continuous_current_limit;  // why not :)
      } else {
        jsd_egd_state_.actual_current = 0;
      }
      break;

    case JSD_EGD_MODE_OF_OPERATION_CST:
      jsd_egd_state_.actual_velocity = copysign(
          jsd_slave_config_.egd.max_motor_speed, jsd_egd_state_.cmd_current);
      jsd_egd_state_.actual_position +=
          jsd_egd_state_.actual_velocity * loop_period_;
      jsd_egd_state_.actual_current = (jsd_motion_cmd_.cst.target_torque_amps +
                                       jsd_motion_cmd_.cst.torque_offset_amps);
      break;

    default:
      ERROR("only CSP, CSV, and CST mode of operations supported");
      return false;
  }

  return true;
}

bool fastcat::EgdOffline::WriteProfiledMode(DeviceCmd& cmd)
{
  switch (cmd.type) {
    case EGD_PROF_POS_CMD: {
      MSG("Prof_pos Write PROF_POS");
      jsd_egd_state_.actual_mode_of_operation =
          JSD_EGD_MODE_OF_OPERATION_PROF_POS;

      jsd_motion_cmd_.prof_pos.target_position =
          cmd.egd_prof_pos_cmd.target_position;
      jsd_motion_cmd_.prof_pos.profile_velocity =
          cmd.egd_prof_pos_cmd.profile_velocity;
      jsd_motion_cmd_.prof_pos.end_velocity = cmd.egd_prof_pos_cmd.end_velocity;
      jsd_motion_cmd_.prof_pos.profile_accel =
          cmd.egd_prof_pos_cmd.profile_accel;
      jsd_motion_cmd_.prof_pos.profile_decel =
          cmd.egd_prof_pos_cmd.profile_decel;
      jsd_motion_cmd_.prof_pos.relative = cmd.egd_prof_pos_cmd.relative;

      // if relative, convert it to absolute
      if (jsd_motion_cmd_.prof_pos.relative) {
        jsd_motion_cmd_.prof_pos.relative = false;
        jsd_motion_cmd_.prof_pos.target_position +=
            jsd_egd_state_.actual_position;
      }

      if (jsd_motion_cmd_.prof_pos.end_velocity != 0) {
        WARNING("%s %s",
                "You have specified a non-zero end_velocity - the EGD will "
                "continue at this",
                "end_velocity even after reaching the target_position.");
      }

      jsd_egd_state_.cmd_position   = jsd_motion_cmd_.prof_pos.target_position;
      jsd_egd_state_.cmd_velocity   = jsd_motion_cmd_.prof_pos.profile_velocity;
      jsd_egd_state_.target_reached = false;
      break;
    }
    case EGD_PROF_VEL_CMD: {
      MSG("Prof_pos Write PROF_VEL");
      jsd_egd_state_.actual_mode_of_operation =
          JSD_EGD_MODE_OF_OPERATION_PROF_VEL;

      jsd_motion_cmd_.prof_vel.target_velocity =
          cmd.egd_prof_vel_cmd.target_velocity;
      jsd_motion_cmd_.prof_vel.profile_accel =
          cmd.egd_prof_vel_cmd.profile_accel;
      jsd_motion_cmd_.prof_vel.profile_decel =
          cmd.egd_prof_vel_cmd.profile_decel;

      jsd_egd_state_.cmd_velocity   = jsd_motion_cmd_.prof_vel.target_velocity;
      jsd_egd_state_.target_reached = false;
      break;
    }
    case EGD_PROF_TORQUE_CMD: {
      MSG("Prof_pos Write PROF_TORQUE");
      jsd_egd_state_.actual_mode_of_operation =
          JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE;

      jsd_motion_cmd_.prof_torque.target_torque_amps =
          cmd.egd_prof_torque_cmd.target_torque_amps;

      jsd_egd_state_.cmd_current =
          jsd_motion_cmd_.prof_torque.target_torque_amps;
      jsd_egd_state_.target_reached = false;
      break;
    }
    case EGD_RESET_CMD: {
      break;
    }
    case EGD_HALT_CMD: {
      jsd_egd_state_.actual_mode_of_operation =
          JSD_EGD_MODE_OF_OPERATION_DISABLED;
      break;
    }
    case EGD_SDO_SET_DRIVE_POS_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_SET_UNIT_MODE_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_DISABLE_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_ENABLE_SPEED_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_ENABLE_POSITION_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    default: {
      WARNING("That command type is not supported in this mode!");
      return false;
    }
  }
  return true;
}

bool fastcat::EgdOffline::WriteCSMode(DeviceCmd& cmd)
{
  switch (cmd.type) {
    case EGD_CSP_CMD: {
      jsd_egd_state_.actual_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_CSP;

      jsd_motion_cmd_.csp.target_position = cmd.egd_csp_cmd.target_position;
      jsd_motion_cmd_.csp.position_offset = cmd.egd_csp_cmd.position_offset;
      jsd_motion_cmd_.csp.velocity_offset = cmd.egd_csp_cmd.velocity_offset;
      jsd_motion_cmd_.csp.torque_offset_amps =
          cmd.egd_csp_cmd.torque_offset_amps;

      jsd_egd_state_.cmd_position = jsd_motion_cmd_.csp.target_position +
                                    jsd_motion_cmd_.csp.position_offset;
      jsd_egd_state_.target_reached = false;
      break;
    }
    case EGD_CSV_CMD: {
      jsd_egd_state_.actual_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_CSV;

      jsd_motion_cmd_.csv.target_velocity = cmd.egd_csv_cmd.target_velocity;
      jsd_motion_cmd_.csv.velocity_offset = cmd.egd_csv_cmd.velocity_offset;
      jsd_motion_cmd_.csv.torque_offset_amps =
          cmd.egd_csv_cmd.torque_offset_amps;

      jsd_egd_state_.cmd_velocity = jsd_motion_cmd_.csv.target_velocity +
                                    jsd_motion_cmd_.csv.velocity_offset;
      jsd_egd_state_.target_reached = false;
      break;
    }
    case EGD_CST_CMD: {
      jsd_egd_state_.actual_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_CST;

      jsd_motion_cmd_.cst.target_torque_amps =
          cmd.egd_cst_cmd.target_torque_amps;
      jsd_motion_cmd_.cst.torque_offset_amps =
          cmd.egd_cst_cmd.torque_offset_amps;

      jsd_egd_state_.cmd_current = jsd_motion_cmd_.cst.target_torque_amps +
                                   jsd_motion_cmd_.cst.torque_offset_amps;
      jsd_egd_state_.target_reached = false;
      break;
    }
    case EGD_RESET_CMD: {
      break;
    }
    case EGD_HALT_CMD: {
      jsd_egd_state_.actual_mode_of_operation =
          JSD_EGD_MODE_OF_OPERATION_DISABLED;
      break;
    }
    case EGD_SET_GAIN_SCHEDULING_INDEX_CMD: {
      WARNING("Offline mode not implemented for manual gain scheduling.");
      break;
    }
    case EGD_SDO_SET_DRIVE_POS_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_SET_UNIT_MODE_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_DISABLE_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_ENABLE_SPEED_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_ENABLE_POSITION_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    case EGD_SDO_ENABLE_MANUAL_GAIN_SCHEDULING_CMD: {
      WARNING("Offline mode not implemented for SDO params");
      break;
    }
    default: {
      ERROR("That command type is not supported in this mode!");
      return false;
    }
  }
  return true;
}
