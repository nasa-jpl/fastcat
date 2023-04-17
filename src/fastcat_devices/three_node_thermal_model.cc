// Include related header (for cc files)
#include "fastcat/fastcat_devices/three_node_thermal_model.h"

namespace fastcat
{
ThreeNodeThermalModel::ThreeNodeThermalModel()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = THREE_NODE_THERMAL_MODEL_STATE;
  // TODO: any other class setup
}

bool ThreeNodeThermalModel::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "thermal_mass_node_1", thermal_mass_node_1_)) {
    return false;
  }

  if (!ParseVal(node, "thermal_mass_node_2", thermal_mass_node_2_)) {
    return false;
  }

  if (!ParseVal(node, "thermal_res_nodes_1_to_2", thermal_res_nodes_1_to_2_)) {
    return false;
  }

  if (!ParseVal(node, "thermal_res_nodes_2_to_3", thermal_res_nodes_2_to_3_)) {
    return false;
  }

  if (!ParseVal(node, "winding_res", winding_res_)) {
    return false;
  }

  if (!ParseVal(node, "winding_thermal_cor", winding_thermal_cor_)) {
    return false;
  }

  if (!ParseVal(node, "k1", k1_)) {
    return false;
  }

  if (!ParseVal(node, "persistance_limit", persistance_limit_)) {
    return false;
  }

  YAML::Node max_allowable_temp_node;
  if (!ParseList(node, "max_allowable_temps", max_allowable_temp_node)) {
    return false;
  }

  if (max_allowable_temp_node.size() != max_allowable_temps_.size()) {
    ERROR("There must be exactly 4 temperature limit values provided");
    return false;
  }

  for (size_t idx = 0; idx < max_allowable_temp_node.size(); ++idx) {
    max_allowable_temps_[idx] = max_allowable_temp_node[idx].as<double>();
  }

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }

  if (signals_.size() < FC_TNTM_NUM_SIGNALS) {
    ERROR("Expecting at least %d signals for Three Node Thermal Model device",
          FC_TNTM_NUM_SIGNALS);
    return false;
  }

  return true;
}

bool Read()
{
  // update signals
  for (size_t idx = 0; idx < FC_TNTM_NUM_SIGNALS; idx++) {
    if (!UpdateSignal(signals_[idx])) {
      ERROR("Could not extract signal");
      return false;
    }
  }

  // store them
  node_3_temp_ =
      signals_[NODE_3_TEMP_IDX].value;  // node 3 temperature is directly taken
                                        // from the signal measurement
  motor_current_ = signals_[MOTOR_CURRENT_IDX].value;
  return true;
}

FaultType Process()
{
  // TODO
  // update motor resistance
  motor_res_ =
      winding_res_ *
      (1 + winding_thermal_cor_ * (node_temps_[0] - REFERENCE_TEMPERATURE));
  // calculate heat transfer rates
  double q_in = motor_current_ * motor_current_ * motor_res_;
  double q_node_1_to_2 =
      (node_temps_[0] - node_temps_[1]) / thermal_res_nodes_1_to_2_;
  double q_node_2_to_3 =
      (node_temps_[1] - node_temps_[2]) / thermal_res_nodes_2_to_3_;
  // calculate temperatures
  node_temps_[0] +=
      (q_in - q_node_1_to_2) * (loop_period_ / thermal_mass_node_1_);
  node_temps_[1] +=
      (q_node_1_to_2 - q_node_2_to_3) * (loop_period_ / thermal_mass_node_2_);
  node_temps_[3] = (k1_ * node_temps_[0] + k3_ * node_temps_[2]) / (k1_ + k3_);
  // update persistence counter for each node
  for (size_t idx = 0; idx < node_temps_.size()) {
    if (node_temps_[idx] > max_allowable_temps_[idx]) {
      node_overtemp_persistences_[idx]++;
    } else {
      node_overtemp_persistences_[idx] = 0;
    }
    // throw fault if limit exceeded
    if (node_overtemp_persistences_[idx] > persistence_limit_) {
      ERROR("Node %d temperature exceeded safety limit -- faulting", idx);
      return ALL_DEVICE_FAULT;
    }
  }
  return NO_FAULT;
}

bool Write(DeviceCmd& cmd)
{
  // TODO
  if (cmd.type == SEED_THERMAL_MODEL_TEMPERATURE_CMD) {
    for (size_t idx = 0; idx < node_temps_.size()) {
      node_temps_[idx] = cmd.seed_thermal_model_temperature.seed_temperature;
    }
  }
  return true;
}

// TODO: add function/action to seed temperature
}  // namespace fastcat
