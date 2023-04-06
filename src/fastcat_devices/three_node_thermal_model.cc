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

  // TODO
}

bool Read() {}

FaultType Process() {}

bool Write(DeviceCmd& cmd) {}
}  // namespace fastcat
