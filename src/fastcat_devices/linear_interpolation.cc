// Include related header (for cc files)
#include "fastcat/fastcat_devices/linear_interpolation.h"

// Include c then c++ libraries
#include <cmath>
#include <cassert>
#include <algorithm>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::LinearInterpolation::LinearInterpolation()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = LINEAR_INTERPOLATION_STATE;
}

bool fastcat::LinearInterpolation::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "enable_out_of_bounds_fault", enable_out_of_bounds_fault_)){
    return false;
  }

  YAML::Node domain_node;
  if (!ParseList(node, "domain", domain_node)) {
    return false;
  }

  YAML::Node range_node;
  if (!ParseList(node, "range", range_node)) {
    return false;
  }

  if(range_node.size() != domain_node.size()){
    ERROR("Range size (%zu) and Domain size (%zu) do no match", 
        range_node.size(), domain_node.size());
    return false;
  }

  if(range_node.size() < 2){
    ERROR("Interpolation Table must consist of at least 2 elements. Provided: (%zu)", 
        range_node.size());
    return false;
  }

  // group together before sorting
  std::pair<double, double> entry;
  auto domain_iter = domain_node.begin();
  auto range_iter = range_node.begin();
  for(size_t i = 0; i < range_node.size(); i++){
    entry.first  = domain_iter->as<double>();
    entry.second = range_iter->as<double>();
    domain_range_.push_back(entry);
    domain_iter++;
    range_iter++;
  }

  // Finally sort them by increasing domain
  std::sort(
    domain_range_.begin(), 
    domain_range_.end(), 
    [](auto &lhs, auto &rhs) { 
      return lhs.first < rhs.first;
      });

  // And precompute the slope
  MSG_DEBUG("Interpolation Table (%s)", name_.c_str());
  MSG_DEBUG("row: domain, range, slope ");
  size_t i;
  for(i = 0; i < domain_range_.size()-1; i++){
    double dy = domain_range_[i+1].second - domain_range_[i].second;
    double dx = domain_range_[i+1].first  - domain_range_[i].first;
    if(fabs(dx) < 1e-16){
      ERROR("Interpolation domain entries are too close (or repeated)");
      return false;
    }
    slope_.push_back(dy/dx);

    MSG_DEBUG("%zu: %lf, %lf, %lf", 
        i, domain_range_[i].first, domain_range_[i].second, slope_[i]);
  }
  MSG_DEBUG("%zu: %lf, %lf, N/A", 
      i, domain_range_[i].first, domain_range_[i].second);

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for LinearInterpolation Device");
    return false;
  }

  return true;
}


bool fastcat::LinearInterpolation::Read() {

  // update input signal
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }

  double signal_value = signals_[0].value;

  size_t i;
  for(i = 0; i < domain_range_.size()-1; i++){
    if(signal_value < domain_range_[i+1].first){
      i++;
      break;
    }
  }
  i--;
  assert(i < domain_range_.size());

  // Check if extrapolating
  if( (i == 0) or (i == (domain_range_.size()-1)) ) {

    // Saturate
    state_->linear_interpolation_state.output = domain_range_[i].second;

    if(enable_out_of_bounds_fault_ and (not device_fault_active_)){
      ERROR("Linear Interpolation (%s) out of interpolation domain (%g to %g) signal was (%g)", 
          name_.c_str(), 
          domain_range_.front().first,
          domain_range_.back().first,
          signal_value);
      return false;
    }
  }else{

    // calc linear interpolation output[i] = y[i] + m[i](input[i] - x[i])
    state_->linear_interpolation_state.output = domain_range_[i].second + 
      slope_[i]*(signal_value - domain_range_[i].first);
  }
  return true;
}
