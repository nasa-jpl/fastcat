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

  if (!ParseVal(node, "thermal_mass_node_1_on", thermal_mass_node_1_on_)) {
    return false;
  }

  if (!ParseVal(node, "thermal_mass_node_1_off", thermal_mass_node_1_off_)) {
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

  if (!ParseOptVal(node, "ref_temp", ref_temp_)) {
    ERROR("ref_temp not found! awaiting_seed_temp_ set to true!!!!");
    awaiting_seed_temp_ = true;
  }
  else {
    awaiting_seed_temp_ = false;
    // initialize all temps to ref_temp
    // Note: this can be manually seeded later
    for (size_t idx = 0; idx < node_temps_.size(); ++idx) {
      node_temps_[idx] = ref_temp_;
    }
  }

  if (!ParseOptVal(node, "exp_smoothing_alpha", exp_smoothing_alpha_)) {
    exp_smoothing_alpha_ = 1.0; // If we set this to 1, it is the same as having no exp. smoothing
  }
  else if (exp_smoothing_alpha_ < 0.0 || exp_smoothing_alpha_ > 1.0) {
    ERROR("Invalid choice for exp_smoothing_alpha! This must be between 0.0 and 1.0 for stability!");
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

  double node_3_temp_sample = signals_[NODE_3_TEMP_IDX].value;

  if (awaiting_seed_temp_) {
    ERROR("AWAITING SEED TEMP IS TRUE... we should have the correct value!!!!"); 
    ERROR("Node temps size is: %d", node_temps_.size());
    for (size_t idx = 0; idx < node_temps_.size(); ++idx) {
      node_temps_[idx] = node_3_temp_sample;
    }
    awaiting_seed_temp_ = false;
  }
  ERROR("Value before smoothing alpha is %f", node_3_temp_sample);
  node_temps_[2] = exp_smoothing_alpha_ * node_3_temp_sample + (1.0 - exp_smoothing_alpha_) * node_temps_[2];
  ERROR("Value after smoothing alpha is %f", node_temps_[2]);
  motor_current_ = signals_[MOTOR_CURRENT_IDX].value;
  motor_on_status_ = signals_[MOTOR_ON_STATUS_IDX].value;
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
  // calculate temperatures
  double thermal_mass_node_1 = motor_on_status_ ? thermal_mass_node_1_on_ : thermal_mass_node_1_off_;

  node_temps_[0] += (q_in - q_node_1_to_2) *
                    ((state_->time - last_time_) / thermal_mass_node_1);
  node_temps_[1] += (q_node_1_to_2 - q_node_2_to_3) *
                    ((state_->time - last_time_) / thermal_mass_node_2_);
  node_temps_[3] =
      (k1_ * node_temps_[0] + k2_ * node_temps_[1] + k3_ * node_temps_[2]) /
      (k1_ + k2_ + k3_);
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
