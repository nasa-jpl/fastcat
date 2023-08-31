// Include related header (for cc files)
#include "fastcat/jsd/ati_fts.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::AtiFts::AtiFts()
{
  MSG_DEBUG("Constructed AtiFts");

  state_       = std::make_shared<DeviceState>();
  state_->type = FTS_STATE;
}

bool fastcat::AtiFts::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_ATI_FTS_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  uint32_t cal;
  if (!ParseVal(node, "calibration", cal)) {
    return false;
  }
  jsd_slave_config_.ati_fts.calibration = cal;

  double dummy;
  if (ParseOptVal(node, "max_force", dummy)) {
    ERROR(
        "fastcat no longer accept L2 norm, configure your yaml values per "
        "axis");
    return false;
  }

  if (!ParseVal(node, "max_force_x", max_force_[0])) {
    return false;
  }

  if (!ParseVal(node, "max_force_y", max_force_[1])) {
    return false;
  }

  if (!ParseVal(node, "max_force_z", max_force_[2])) {
    return false;
  }

  if (!ParseVal(node, "max_torque_x", max_torque_[0])) {
    return false;
  }

  if (!ParseVal(node, "max_torque_y", max_torque_[1])) {
    return false;
  }

  if (!ParseVal(node, "max_torque_z", max_torque_[2])) {
    return false;
  }

  return true;
}

bool fastcat::AtiFts::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::AtiFts::Read()
{
  jsd_ati_fts_read((jsd_t*)context_, slave_id_);

  const jsd_ati_fts_state_t* ati_fts_state;
  ati_fts_state = jsd_ati_fts_get_state((jsd_t*)context_, slave_id_);

  state_->fts_state.raw_fx = ati_fts_state->fx;
  state_->fts_state.raw_fy = ati_fts_state->fy;
  state_->fts_state.raw_fz = ati_fts_state->fz;
  state_->fts_state.raw_tx = ati_fts_state->tx;
  state_->fts_state.raw_ty = ati_fts_state->ty;
  state_->fts_state.raw_tz = ati_fts_state->tz;

  state_->fts_state.tared_fx = ati_fts_state->fx + bias_[0];
  state_->fts_state.tared_fy = ati_fts_state->fy + bias_[1];
  state_->fts_state.tared_fz = ati_fts_state->fz + bias_[2];
  state_->fts_state.tared_tx = ati_fts_state->tx + bias_[3];
  state_->fts_state.tared_ty = ati_fts_state->ty + bias_[4];
  state_->fts_state.tared_tz = ati_fts_state->tz + bias_[5];

  ati_error_       = ati_fts_state->active_error;
  ati_status_code_ = ati_fts_state->status_code;

  return true;
}

fastcat::FaultType fastcat::AtiFts::Process()
{
  jsd_ati_fts_process((jsd_t*)context_, slave_id_);

  if (!device_fault_active_) {
    if (ati_error_) {
      ERROR("ATI FTS reports internal error: Status Code = %u",
            ati_status_code_);
      return ALL_DEVICE_FAULT;
    }

    if (enable_fts_guard_fault_) {
      if (max_force_[0] < fabs(state_->fts_state.raw_fx) ||
          max_force_[1] < fabs(state_->fts_state.raw_fy) ||
          max_force_[2] < fabs(state_->fts_state.raw_fz) ||
          max_torque_[0] < fabs(state_->fts_state.raw_tx) ||
          max_torque_[1] < fabs(state_->fts_state.raw_ty) ||
          max_torque_[2] < fabs(state_->fts_state.raw_tz)) {
        ERROR(
            "Force or torque measured by device %s exceeded maximum allowable "
            "magnitude. Force: [x]: %f / %f, [y]: %f / %f, [z]: %f / %f "
            "Torque: [x]: %f / %f, [y]: %f / %f, [z]: %f / %f",
            name_.c_str(), state_->fts_state.raw_fx, max_force_[0],
            state_->fts_state.raw_fy, max_force_[1], state_->fts_state.raw_fz,
            max_force_[2], state_->fts_state.raw_tx, max_torque_[0],
            state_->fts_state.raw_ty, max_torque_[1], state_->fts_state.raw_tz,
            max_torque_[2]);
        return ALL_DEVICE_FAULT;
      }
    }
  }

  return NO_FAULT;
}

bool fastcat::AtiFts::Write(DeviceCmd& cmd)
{
  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if (sdoResult != SDO_RET_VAL_NOT_APPLICABLE) {
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  if (cmd.type == FTS_TARE_CMD) {
    // Do not permit taring if there is a fault
    if (device_fault_active_) {
      ERROR("Taring (%s) FTS is not permited with an active fault, reset first",
            name_.c_str());
      return false;
    } else {
      bias_[0] = -state_->fts_state.raw_fx;
      bias_[1] = -state_->fts_state.raw_fy;
      bias_[2] = -state_->fts_state.raw_fz;
      bias_[3] = -state_->fts_state.raw_tx;
      bias_[4] = -state_->fts_state.raw_ty;
      bias_[5] = -state_->fts_state.raw_tz;
      return true;
    }
  } else if (cmd.type == FTS_ENABLE_GUARD_FAULT_CMD) {
    enable_fts_guard_fault_ = cmd.fts_enable_guard_fault_cmd.enable;
    return true;
  } else {
    WARNING("That command type is not supported!");
    return false;
  }
}
