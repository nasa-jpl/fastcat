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

bool fastcat::AtiFts::ConfigFromYamlCommon(YAML::Node node)
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

  if (!ParseVal(node, "max_force", max_force_)) {
    return false;
  }

  if (!ParseVal(node, "max_torque", max_torque_)) {
    return false;
  }

  return true;
}

bool fastcat::AtiFts::ConfigFromYaml(YAML::Node node)
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

    double force_mag  = sqrt(pow(state_->fts_state.tared_fx, 2) +
                            pow(state_->fts_state.tared_fy, 2) +
                            pow(state_->fts_state.tared_fz, 2));
    double torque_mag = sqrt(pow(state_->fts_state.tared_tx, 2) +
                             pow(state_->fts_state.tared_ty, 2) +
                             pow(state_->fts_state.tared_tz, 2));
    if (force_mag > max_force_ || torque_mag > max_torque_) {
      ERROR(
          "Force or torque measured by device %s exceeded maximum allowable "
          "magnitude. Force: %f / %f "
          "Torque: %f / %f",
          name_.c_str(), force_mag, max_force_, torque_mag, max_torque_);
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

bool fastcat::AtiFts::Write(DeviceCmd& cmd)
{
  if (cmd.type != FTS_TARE_CMD) {
    WARNING("That command type is not supported!");
    return false;
  }

  bias_[0] = -state_->fts_state.raw_fx;
  bias_[1] = -state_->fts_state.raw_fy;
  bias_[2] = -state_->fts_state.raw_fz;
  bias_[3] = -state_->fts_state.raw_tx;
  bias_[4] = -state_->fts_state.raw_ty;
  bias_[5] = -state_->fts_state.raw_tz;

  return true;
}
