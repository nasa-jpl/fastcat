// Include related header (for cc files)
#include "fastcat/fastcat_devices/virtual_fts.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::VirtualFts::VirtualFts()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = FTS_STATE;
}

bool fastcat::VirtualFts::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  // Position
  YAML::Node pos_node;
  if (!ParseList(node, "position", pos_node)) {
    return false;
  }

  if (pos_node.size() != 3) {
    ERROR("Position input does not contain 3 elements");
    return false;
  }

  tf_.position = {pos_node[0].as<double>(), pos_node[1].as<double>(),
                  pos_node[2].as<double>()};

  // Rotation (quaternion or Euler)
  YAML::Node rot_node;
  if (ParseList(node, "quaternion", rot_node)) {
    // Quaternion case
    if (rot_node.size() != 4) {
      ERROR("Rotation input is labeled as a quaternion, but has %zu elements",
            rot_node.size());
      return false;
    }

    tf_.rotation = {rot_node[0].as<double>(), rot_node[1].as<double>(),
                    rot_node[2].as<double>(), rot_node[3].as<double>()};
  } else if (ParseList(node, "euler", rot_node)) {
    // Euler case
    if (rot_node.size() != 3) {
      ERROR("Rotation input is labeled as Euler angles, but has %zu elements",
            rot_node.size());
      return false;
    }

    tf_.rotation =
        QuatFromRpy(rot_node[0].as<double>(), rot_node[1].as<double>(),
                    rot_node[2].as<double>());
  } else {
    return false;
  }

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

  // Signals
  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }

  if (signals_.size() < FC_FTS_N_DIMS) {
    ERROR("Expecting exactly %d signals for VirtualFts", FC_FTS_N_DIMS);
    return false;
  }

  return true;
}

bool fastcat::VirtualFts::Read()
{
  for (int ii = 0; ii < FC_FTS_N_DIMS; ii++) {
    if (!UpdateSignal(signals_[ii])) {
      ERROR("Could not extract signal");
      return false;
    }

    wrench_[ii] = signals_[ii].value;
  }

  wrench init_wrench = {{wrench_[0], wrench_[1], wrench_[2]},
                        {wrench_[3], wrench_[4], wrench_[5]}};

  wrench tf_wrench = WrenchTransform(init_wrench, tf_);

  wrench_[0] = tf_wrench.forces.x;
  wrench_[1] = tf_wrench.forces.y;
  wrench_[2] = tf_wrench.forces.z;
  wrench_[3] = tf_wrench.torques.x;
  wrench_[4] = tf_wrench.torques.y;
  wrench_[5] = tf_wrench.torques.z;

  state_->fts_state.raw_fx = tf_wrench.forces.x;
  state_->fts_state.raw_fy = tf_wrench.forces.y;
  state_->fts_state.raw_fz = tf_wrench.forces.z;
  state_->fts_state.raw_tx = tf_wrench.torques.x;
  state_->fts_state.raw_ty = tf_wrench.torques.y;
  state_->fts_state.raw_tz = tf_wrench.torques.z;

  state_->fts_state.tared_fx = tf_wrench.forces.x + sig_offset_[0];
  state_->fts_state.tared_fy = tf_wrench.forces.y + sig_offset_[1];
  state_->fts_state.tared_fz = tf_wrench.forces.z + sig_offset_[2];
  state_->fts_state.tared_tx = tf_wrench.torques.x + sig_offset_[3];
  state_->fts_state.tared_ty = tf_wrench.torques.y + sig_offset_[4];
  state_->fts_state.tared_tz = tf_wrench.torques.z + sig_offset_[5];

  return true;
}
