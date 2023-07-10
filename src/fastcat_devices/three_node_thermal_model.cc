// Include related header (for cc files)
#include "fastcat/fastcat_devices/three_node_thermal_model.h"

namespace fastcat
{
ThreeNodeThermalModel::ThreeNodeThermalModel()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = THREE_NODE_THERMAL_MODEL_STATE;
  last_time_   = jsd_time_get_time_sec();  // init time
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

  if (!ParseVal(node, "k2", k2_)) {
    return false;
  }

  if (!ParseVal(node, "k3", k3_)) {
    return false;
  }

  if (!ParseVal(node, "persistence_limit", persistence_limit_)) {
    return false;
  }

  if (!ParseVal(node, "ref_temp", ref_temp_)) {
    return false;
  }
  // initialize all temps to ref_temp
  // Note: this can be manually seeded later
  for (size_t idx = 0; idx < node_temps_.size(); ++idx) {
    node_temps_[idx] = ref_temp_;
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

  if (signals_.size() != FC_TNTM_NUM_SIGNALS) {
    ERROR("Expecting exactly %ld signals for Three Node Thermal Model device",
          FC_TNTM_NUM_SIGNALS);
    return false;
  }

  return true;
}

bool ThreeNodeThermalModel::Read()
{
  // update signals
  for (size_t idx = 0; idx < FC_TNTM_NUM_SIGNALS; idx++) {
    if (!UpdateSignal(signals_[idx])) {
      ERROR("Could not extract signal");
      return false;
    }
  }

  // store them
  node_temps_[2] =
      signals_[NODE_3_TEMP_IDX].value;  // node 3 temperature is directly taken
                                        // from the signal measurement
  motor_current_ = signals_[MOTOR_CURRENT_IDX].value;
  return true;
}

FaultType ThreeNodeThermalModel::Process()
{
  // TODO should there be a check/flag that's monitored for if the model has
  // been initialized/seeded update motor resistance
  motor_res_ =
      winding_res_ * (1 + winding_thermal_cor_ * (node_temps_[0] - ref_temp_));
  // calculate heat transfer rates
  double q_in = motor_current_ * motor_current_ * motor_res_;  // I^2 * R
  double q_node_1_to_2 =
      (node_temps_[0] - node_temps_[1]) / thermal_res_nodes_1_to_2_;
  double q_node_2_to_3 =
      (node_temps_[1] - node_temps_[2]) / thermal_res_nodes_2_to_3_;
  auto time_difference = state_->time - last_time_;
  ERROR("\n\ntime_difference: %d", time_difference)
  ERROR("q_node_1_to_2: %f", q_node_1_to_2);
  ERROR("q_node_2_to_3: %f", q_node_2_to_3);
  // calculate temperatures
  ERROR("node_temps[0] before addition: %f", node_temps_[0]);
  ERROR("node_temps[1] before addition: %f", node_temps_[1]);
  ERROR("node_temps[2] sensor reading: %f", node_temps_[2]);
  ERROR("node_temps[3] before addition: %f", node_temps_[3]);
  node_temps_[0] += (q_in - q_node_1_to_2) *
                    ((state_->time - last_time_) / thermal_mass_node_1_);
  node_temps_[1] += (q_node_1_to_2 - q_node_2_to_3) *
                    ((state_->time - last_time_) / thermal_mass_node_2_);
  node_temps_[3] =
      (k1_ * node_temps_[0] + k2_ * node_temps_[1] + k3_ * node_temps_[2]) /
      (k1_ + k2_ + k3_);
  ERROR("node_temps[0] after addition: %f", node_temps_[0]);
  ERROR("node_temps[1] after addition: %f", node_temps_[1]);
  ERROR("node_temps[2] sensor reading: %f", node_temps_[2]);
  ERROR("node_temps[3] after addition: %f", node_temps_[3]);
  // update persistence counter for each node
  for (size_t idx = 0; idx < node_temps_.size(); ++idx) {
    if (node_temps_[idx] > max_allowable_temps_[idx]) {
      node_overtemp_persistences_[idx]++;
    } else {
      node_overtemp_persistences_[idx] = 0;
    }
    // throw fault if limit exceeded
    if (node_overtemp_persistences_[idx] > persistence_limit_) {
      ERROR("Node %ld temperature (%f degrees Celsius) exceeded safety limit -- faulting", idx + 1u, node_temps_[idx]);
      return ALL_DEVICE_FAULT;
    }
  }
  state_->three_node_thermal_model_state.node_1_temp = node_temps_[0];
  state_->three_node_thermal_model_state.node_2_temp = node_temps_[1];
  state_->three_node_thermal_model_state.node_3_temp = node_temps_[2];
  state_->three_node_thermal_model_state.node_4_temp = node_temps_[3];
  last_time_ = state_->time;  // update loop time

  return NO_FAULT;
}

bool ThreeNodeThermalModel::Write(DeviceCmd& cmd)
{
  if (cmd.type == SEED_THERMAL_MODEL_TEMPERATURE_CMD) {
    for (size_t idx = 0; idx < node_temps_.size(); ++idx) {
      node_temps_[idx] =
          cmd.seed_thermal_model_temperature_cmd.seed_temperature;
    }
  } else {
    return false;  // if command is not present, return false
  }
  return true;
}
}  // namespace fastcat
