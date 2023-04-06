// Include related header (for cc files)
#include "fastcat/fastcat_devices/three_node_thermal_model.h"

namespace fastcat
{
ThreeNodeThermalModel::Fts()
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

  // TODO: validate signal size proviced in yaml
  return true;
}

bool Read() {}

FaultType Process() {}

bool Write(DeviceCmd& cmd) {}
}  // namespace fastcat
