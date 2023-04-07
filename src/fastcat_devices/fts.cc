// Include related header (for cc files)
#include "fastcat/fastcat_devices/fts.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Fts::Fts()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = FTS_STATE;
}

bool fastcat::Fts::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

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

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }

  if (signals_.size() < FC_FTS_N_DIMS) {
    ERROR("Expecting at least %d signals for FTS", FC_FTS_N_DIMS);
    return false;
  }

  YAML::Node calib_node;
  if (!ParseList(node, "calibration_matrix", calib_node)) {
    return false;
  }

  if (calib_node.size() % FC_FTS_N_DIMS != 0) {
    ERROR("Calibration matrix size (%lu) not a multiple of %d",
          calib_node.size(), FC_FTS_N_DIMS);
    return false;
  }

  calibration_.resize(FC_FTS_N_DIMS, std::vector<double>(signals_.size(), 0));

  size_t el = 0;
  for (auto cc = calib_node.begin(); cc != calib_node.end(); ++cc) {
    calibration_[el / signals_.size()][el % signals_.size()] = (*cc).as<double>();
    el++;
  }

  // print the callibration matrix
  std::string cal_mat_str("FTS Cal Matrix = \n");
  for (int i = 0; i < FC_FTS_N_DIMS; ++i) {
    cal_mat_str.append("\t");
    for (size_t j = 0; j < signals_.size(); ++j) {
      char term[64];
      snprintf(term, 64, " %lf ", calibration_[i][j]);
      cal_mat_str.append(term);
    }
    cal_mat_str.append("\n");
  }
  MSG_DEBUG("%s", cal_mat_str.c_str());

  return true;
}

bool fastcat::Fts::Read()
{
  int i= 0;
  for (auto& signal : signals_) {
    if (!UpdateSignal(signal)) {
      ERROR("Could not extract signal");
      return false;
    }
    MSG_DEBUG("Signal number %d is value %f", i, signal.value);
    i++;
  }

  for (int ii = 0; ii < FC_FTS_N_DIMS; ii++) {
    wrench_[ii] = 0;

    for (size_t jj = 0; jj < signals_.size(); jj++) {
      wrench_[ii] += calibration_[ii][jj] * signals_[jj].value;
    }
  }

  state_->fts_state.raw_fx   = wrench_[0];
  state_->fts_state.raw_fy   = wrench_[1];
  state_->fts_state.raw_fz   = wrench_[2];
  state_->fts_state.raw_tx   = wrench_[3];
  state_->fts_state.raw_ty   = wrench_[4];
  state_->fts_state.raw_tz   = wrench_[5];
  state_->fts_state.tared_fx = wrench_[0] + sig_offset_[0];
  state_->fts_state.tared_fy = wrench_[1] + sig_offset_[1];
  state_->fts_state.tared_fz = wrench_[2] + sig_offset_[2];
  state_->fts_state.tared_tx = wrench_[3] + sig_offset_[3];
  state_->fts_state.tared_ty = wrench_[4] + sig_offset_[4];
  state_->fts_state.tared_tz = wrench_[5] + sig_offset_[5];

  return true;
}

fastcat::FaultType fastcat::Fts::Process()
{
  if (!device_fault_active_) {
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
bool fastcat::Fts::Write(DeviceCmd& cmd)
{
  if (cmd.type == FTS_TARE_CMD) {
    // Do not permit taring if there is a fault
    if (device_fault_active_) {
      ERROR("Taring FTS is not permited with a active fault, reset first");
      return false;
    } else {
      for (int ii = 0; ii < FC_FTS_N_DIMS; ii++) {
        sig_offset_[ii] = -wrench_[ii];
      }
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
