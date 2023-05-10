// Include related header (for cc files)
#include "fastcat/jsd/egd.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::Egd::Egd()
{
  MSG_DEBUG("Constructed Egd");

  state_       = std::make_shared<DeviceState>();
  state_->type = EGD_DEVICE;
}

bool fastcat::Egd::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config(context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::Egd::Read()
{
  jsd_egd_read(context_, slave_id_);

  memcpy(&jsd_egd_state_, jsd_egd_get_state(context_, slave_id_),
         sizeof(jsd_egd_state_t));

  // copy signal data
  state_->egd_state.actual_position = jsd_egd_state_.actual_position;
  state_->egd_state.actual_velocity = jsd_egd_state_.actual_velocity;
  state_->egd_state.actual_current  = jsd_egd_state_.actual_current;
  state_->egd_state.cmd_position    = jsd_egd_state_.cmd_position;
  state_->egd_state.cmd_velocity    = jsd_egd_state_.cmd_velocity;
  state_->egd_state.cmd_current     = jsd_egd_state_.cmd_current;
  state_->egd_state.cmd_ff_position = jsd_egd_state_.cmd_ff_position;
  state_->egd_state.cmd_ff_velocity = jsd_egd_state_.cmd_ff_velocity;
  state_->egd_state.cmd_ff_current  = jsd_egd_state_.cmd_ff_current;

  state_->egd_state.actual_state_machine_state =
      static_cast<uint32_t>(jsd_egd_state_.actual_state_machine_state);
  state_->egd_state.actual_mode_of_operation =
      static_cast<uint32_t>(jsd_egd_state_.actual_mode_of_operation);

  state_->egd_state.sto_engaged    = jsd_egd_state_.sto_engaged;
  state_->egd_state.hall_state     = jsd_egd_state_.hall_state;
  state_->egd_state.in_motion      = jsd_egd_state_.in_motion;
  state_->egd_state.warning        = jsd_egd_state_.warning;
  state_->egd_state.target_reached = jsd_egd_state_.target_reached;
  state_->egd_state.motor_on       = jsd_egd_state_.motor_on;
  state_->egd_state.servo_enabled  = jsd_egd_state_.servo_enabled;

  state_->egd_state.emcy_error_code = jsd_egd_state_.emcy_error_code;
  state_->egd_state.fault_code =
      static_cast<uint32_t>(jsd_egd_state_.fault_code);

  state_->egd_state.bus_voltage          = jsd_egd_state_.bus_voltage;
  state_->egd_state.analog_input_voltage = jsd_egd_state_.analog_input_voltage;
  state_->egd_state.digital_input_ch1    = jsd_egd_state_.digital_inputs[0];
  state_->egd_state.digital_input_ch2    = jsd_egd_state_.digital_inputs[1];
  state_->egd_state.digital_input_ch3    = jsd_egd_state_.digital_inputs[2];
  state_->egd_state.digital_input_ch4    = jsd_egd_state_.digital_inputs[3];
  state_->egd_state.digital_input_ch5    = jsd_egd_state_.digital_inputs[4];
  state_->egd_state.digital_input_ch6    = jsd_egd_state_.digital_inputs[5];

  state_->egd_state.digital_output_cmd_ch1 =
      jsd_egd_state_.digital_output_cmd[0];
  state_->egd_state.digital_output_cmd_ch2 =
      jsd_egd_state_.digital_output_cmd[1];
  state_->egd_state.digital_output_cmd_ch3 =
      jsd_egd_state_.digital_output_cmd[2];
  state_->egd_state.digital_output_cmd_ch4 =
      jsd_egd_state_.digital_output_cmd[3];
  state_->egd_state.digital_output_cmd_ch5 =
      jsd_egd_state_.digital_output_cmd[4];
  state_->egd_state.digital_output_cmd_ch6 =
      jsd_egd_state_.digital_output_cmd[5];

  state_->egd_state.drive_temperature = jsd_egd_state_.drive_temperature;

  state_->egd_state.faulted = (jsd_egd_state_.fault_code != JSD_EGD_FAULT_OKAY);

  return true;
}

fastcat::FaultType fastcat::Egd::Process()
{
  jsd_egd_process(context_, slave_id_);
  return NO_FAULT;
}

bool fastcat::Egd::Write(DeviceCmd& cmd)
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

void fastcat::Egd::Fault()
{
  DeviceBase::Fault();
  jsd_egd_halt(context_, slave_id_);
}

void fastcat::Egd::Reset()
{
  DeviceBase::Reset();
  jsd_egd_reset(context_, slave_id_);
}

bool fastcat::Egd::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EGD_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  if (!ParseVal(node, "drive_cmd_mode", drive_cmd_mode_string_)) {
    return false;
  }

  if (!DriveCmdModeFromString(drive_cmd_mode_string_,
                              jsd_slave_config_.egd.drive_cmd_mode)) {
    return false;
  }

  if (!ParseVal(node, "cs_cmd_freq_hz", cs_cmd_freq_hz_)) {
    return false;
  }
  if (cs_cmd_freq_hz_ < 1 || cs_cmd_freq_hz_ > 1000) {
    ERROR("cs_cmd_freq_hz(%lf) needs to be greater than 1 and less than 1000",
          cs_cmd_freq_hz_);
    return false;
  }
  jsd_slave_config_.egd.loop_period_ms = 1000.0 / cs_cmd_freq_hz_;

  if (!ParseVal(node, "max_motor_speed",
                jsd_slave_config_.egd.max_motor_speed)) {
    return false;
  }

  if (!ParseVal(node, "torque_slope", jsd_slave_config_.egd.torque_slope)) {
    return false;
  }

  if (!ParseVal(node, "max_profile_accel",
                jsd_slave_config_.egd.max_profile_accel)) {
    return false;
  }

  if (!ParseVal(node, "max_profile_decel",
                jsd_slave_config_.egd.max_profile_decel)) {
    return false;
  }

  if (!ParseVal(node, "velocity_tracking_error",
                jsd_slave_config_.egd.velocity_tracking_error)) {
    return false;
  }

  if (!ParseVal(node, "position_tracking_error",
                jsd_slave_config_.egd.position_tracking_error)) {
    return false;
  }

  if (!ParseVal(node, "peak_current_limit",
                jsd_slave_config_.egd.peak_current_limit)) {
    return false;
  }

  if (!ParseVal(node, "peak_current_time",
                jsd_slave_config_.egd.peak_current_time)) {
    return false;
  }

  if (!ParseVal(node, "continuous_current_limit",
                jsd_slave_config_.egd.continuous_current_limit)) {
    return false;
  }

  if (!ParseVal(node, "motor_stuck_current_level_pct",
                jsd_slave_config_.egd.motor_stuck_current_level_pct)) {
    return false;
  }

  if (!ParseVal(node, "motor_stuck_velocity_threshold",
                jsd_slave_config_.egd.motor_stuck_velocity_threshold)) {
    return false;
  }

  if (!ParseVal(node, "motor_stuck_timeout",
                jsd_slave_config_.egd.motor_stuck_timeout)) {
    return false;
  }

  if (!ParseVal(node, "over_speed_threshold",
                jsd_slave_config_.egd.over_speed_threshold)) {
    return false;
  }

  if (!ParseVal(node, "low_position_limit",
                jsd_slave_config_.egd.low_position_limit)) {
    return false;
  }

  if (!ParseVal(node, "high_position_limit",
                jsd_slave_config_.egd.high_position_limit)) {
    return false;
  }

  if (!ParseVal(node, "brake_engage_msec",
                jsd_slave_config_.egd.brake_engage_msec)) {
    return false;
  }

  if (!ParseVal(node, "brake_disengage_msec",
                jsd_slave_config_.egd.brake_disengage_msec)) {
    return false;
  }

  if (!ParseVal(node, "crc", jsd_slave_config_.egd.crc)) {
    return false;
  }

  if (!ParseVal(node, "drive_max_current_limit",
                jsd_slave_config_.egd.drive_max_current_limit)) {
    return false;
  }

  std::string ctrl_gs_mode_string;
  if (ParseOptVal(node, "ctrl_gain_scheduling_mode", ctrl_gs_mode_string)) {
    if (!GSModeFromString(ctrl_gs_mode_string,
                          jsd_slave_config_.egd.ctrl_gain_scheduling_mode)) {
      return false;
    }
  } else {
    // Use mode saved in driver's non-volatile memory.
    jsd_slave_config_.egd.ctrl_gain_scheduling_mode =
        JSD_ELMO_GAIN_SCHEDULING_MODE_PRELOADED;
  }

  return true;
}

bool fastcat::Egd::DriveCmdModeFromString(std::string               dcm_string,
                                          jsd_egd_drive_cmd_mode_t& dcm)
{
  MSG("Converting drive command mode to string.");
  if (dcm_string.compare("CS") == 0) {
    dcm = JSD_EGD_DRIVE_CMD_MODE_CS;
  } else if (dcm_string.compare("PROFILED") == 0) {
    dcm = JSD_EGD_DRIVE_CMD_MODE_PROFILED;
  } else {
    ERROR("%s is not a valid drive command mode for Egd devices.",
          dcm_string.c_str());
    return false;
  }

  return true;
}

bool fastcat::Egd::WriteProfiledMode(DeviceCmd& cmd)
{
  switch (cmd.type) {
    case EGD_PROF_POS_CMD: {
      jsd_elmo_motion_command_prof_pos_t jsd_cmd;
      jsd_cmd.target_position  = cmd.egd_prof_pos_cmd.target_position;
      jsd_cmd.profile_velocity = cmd.egd_prof_pos_cmd.profile_velocity;
      jsd_cmd.end_velocity     = cmd.egd_prof_pos_cmd.end_velocity;
      jsd_cmd.profile_accel    = cmd.egd_prof_pos_cmd.profile_accel;
      jsd_cmd.profile_decel    = cmd.egd_prof_pos_cmd.profile_decel;
      jsd_cmd.relative         = cmd.egd_prof_pos_cmd.relative;

      jsd_egd_set_motion_command_prof_pos(context_, slave_id_, jsd_cmd);
      break;
    }
    case EGD_PROF_VEL_CMD: {
      jsd_elmo_motion_command_prof_vel_t jsd_cmd;
      jsd_cmd.target_velocity = cmd.egd_prof_vel_cmd.target_velocity;
      jsd_cmd.profile_accel   = cmd.egd_prof_vel_cmd.profile_accel;
      jsd_cmd.profile_decel   = cmd.egd_prof_vel_cmd.profile_decel;

      jsd_egd_set_motion_command_prof_vel(context_, slave_id_, jsd_cmd);
      break;
    }
    case EGD_PROF_TORQUE_CMD: {
      jsd_elmo_motion_command_prof_torque_t jsd_cmd;
      jsd_cmd.target_torque_amps = cmd.egd_prof_torque_cmd.target_torque_amps;

      jsd_egd_set_motion_command_prof_torque(context_, slave_id_, jsd_cmd);
      break;
    }
    case EGD_RESET_CMD: {
      jsd_egd_reset(context_, slave_id_);
      break;
    }
    case EGD_HALT_CMD: {
      jsd_egd_halt(context_, slave_id_);
      break;
    }
    case EGD_SDO_SET_DRIVE_POS_CMD: {
      jsd_egd_async_sdo_set_drive_position(
          context_, slave_id_, cmd.egd_sdo_set_drive_pos_cmd.drive_position,
          cmd.egd_sdo_set_drive_pos_cmd.app_id);
      break;
    }
    case EGD_SDO_SET_UNIT_MODE_CMD: {
      jsd_egd_async_sdo_set_unit_mode(context_, slave_id_,
                                      cmd.egd_sdo_set_unit_mode_cmd.unit_mode,
                                      cmd.egd_sdo_set_unit_mode_cmd.app_id);
      break;
    }
    case EGD_SDO_DISABLE_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_DISABLED,
          cmd.egd_sdo_disable_gain_scheduling_cmd.app_id);
      break;
    }
    case EGD_SDO_ENABLE_SPEED_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_SPEED,
          cmd.egd_sdo_enable_speed_gain_scheduling_cmd.app_id);
      break;
    }
    case EGD_SDO_ENABLE_POSITION_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_POSITION,
          cmd.egd_sdo_enable_position_gain_scheduling_cmd.app_id);
      break;
    }
    case EGD_SDO_ENABLE_MANUAL_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_MANUAL_LOW,
          cmd.egd_sdo_enable_position_gain_scheduling_cmd.app_id);
      break;
    }
    default: {
      WARNING("That command type is not supported in this mode!");
      return false;
    }
  }
  return true;
}

bool fastcat::Egd::WriteCSMode(DeviceCmd& cmd)
{
  switch (cmd.type) {
    case EGD_CSP_CMD: {
      jsd_elmo_motion_command_csp_t jsd_cmd;
      jsd_cmd.target_position    = cmd.egd_csp_cmd.target_position;
      jsd_cmd.position_offset    = cmd.egd_csp_cmd.position_offset;
      jsd_cmd.velocity_offset    = cmd.egd_csp_cmd.velocity_offset;
      jsd_cmd.torque_offset_amps = cmd.egd_csp_cmd.torque_offset_amps;

      jsd_egd_set_motion_command_csp(context_, slave_id_, jsd_cmd);
      break;
    }
    case EGD_CSV_CMD: {
      jsd_elmo_motion_command_csv_t jsd_cmd;
      jsd_cmd.target_velocity    = cmd.egd_csv_cmd.target_velocity;
      jsd_cmd.velocity_offset    = cmd.egd_csv_cmd.velocity_offset;
      jsd_cmd.torque_offset_amps = cmd.egd_csv_cmd.torque_offset_amps;

      jsd_egd_set_motion_command_csv(context_, slave_id_, jsd_cmd);
      break;
    }
    case EGD_CST_CMD: {
      jsd_elmo_motion_command_cst_t jsd_cmd = {0};
      jsd_cmd.target_torque_amps           = cmd.egd_cst_cmd.target_torque_amps;
      jsd_cmd.torque_offset_amps           = cmd.egd_cst_cmd.torque_offset_amps;

      jsd_egd_set_motion_command_cst(context_, slave_id_, jsd_cmd);
      break;
    }
    case EGD_SET_GAIN_SCHEDULING_INDEX_CMD: {
      jsd_egd_set_gain_scheduling_index(
          context_, slave_id_, true,
          cmd.egd_set_gain_scheduling_index_cmd.gain_scheduling_index);
      break;
    }
    case EGD_RESET_CMD: {
      jsd_egd_reset(context_, slave_id_);
      break;
    }
    case EGD_HALT_CMD: {
      jsd_egd_halt(context_, slave_id_);
      break;
    }
    case EGD_SDO_SET_DRIVE_POS_CMD: {
      jsd_egd_async_sdo_set_drive_position(
          context_, slave_id_, cmd.egd_sdo_set_drive_pos_cmd.drive_position,
          cmd.egd_sdo_set_drive_pos_cmd.app_id);
      break;
    }
    case EGD_SDO_SET_UNIT_MODE_CMD: {
      jsd_egd_async_sdo_set_unit_mode(context_, slave_id_,
                                      cmd.egd_sdo_set_unit_mode_cmd.unit_mode,
                                      cmd.egd_sdo_set_unit_mode_cmd.app_id);
      break;
    }
    case EGD_SDO_DISABLE_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_DISABLED,
          cmd.egd_sdo_disable_gain_scheduling_cmd.app_id);
      break;
    }
    case EGD_SDO_ENABLE_SPEED_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_SPEED,
          cmd.egd_sdo_enable_speed_gain_scheduling_cmd.app_id);
      break;
    }
    case EGD_SDO_ENABLE_POSITION_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_POSITION,
          cmd.egd_sdo_enable_position_gain_scheduling_cmd.app_id);
      break;
    }
    case EGD_SDO_ENABLE_MANUAL_GAIN_SCHEDULING_CMD: {
      jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
          context_, slave_id_, JSD_ELMO_GAIN_SCHEDULING_MODE_MANUAL_LOW,
          cmd.egd_sdo_enable_position_gain_scheduling_cmd.app_id);
      break;
    }
    default: {
      ERROR("That command type is not supported in this mode!");
      return false;
    }
  }
  return true;
}

bool fastcat::Egd::GSModeFromString(std::string gs_mode_string,
                                    jsd_elmo_gain_scheduling_mode_t& gs_mode)
{
  MSG("Converting gain scheduling mode to string.");
  if (gs_mode_string.compare("DISABLED") == 0) {
    gs_mode = JSD_ELMO_GAIN_SCHEDULING_MODE_DISABLED;
  } else if (gs_mode_string.compare("SPEED") == 0) {
    gs_mode = JSD_ELMO_GAIN_SCHEDULING_MODE_SPEED;
  } else if (gs_mode_string.compare("POSITION") == 0) {
    gs_mode = JSD_ELMO_GAIN_SCHEDULING_MODE_POSITION;
  } else if (gs_mode_string.compare("MANUAL") == 0) {
    gs_mode = JSD_ELMO_GAIN_SCHEDULING_MODE_MANUAL_LOW;
  } else {
    ERROR("Gain scheduling mode %s is invalid", gs_mode_string.c_str());
    return false;
  }

  return true;
}
