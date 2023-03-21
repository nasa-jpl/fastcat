// Include related header (for cc files)
#include "fastcat/fastcat_devices/linear_interpolation.h"

// Include c then c++ libraries
#include <algorithm>
#include <cassert>
#include <cmath>

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

  if (!ParseVal(node, "enable_out_of_bounds_fault",
                enable_out_of_bounds_fault_)) {
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

  if (range_node.size() < 2) {
    ERROR(
        "Interpolation Table must consist of at least 2 elements. Provided: "
        "(%zu)",
        range_node.size());
    return false;
  }

  if (range_node.size() != domain_node.size()) {
    ERROR("Range size (%zu) and Domain size (%zu) do no match",
          range_node.size(), domain_node.size());
    return false;
  }
  domain_range_.resize(range_node.size());

  // group together before sorting
  auto domain_iter = domain_node.begin();
  auto range_iter  = range_node.begin();
  for (size_t i = 0; i < range_node.size(); i++) {
    domain_range_[i].first  = domain_iter->as<double>();
    domain_range_[i].second = range_iter->as<double>();
    domain_iter++;
    range_iter++;
  }

  // Finally sort them by increasing domain
  std::sort(domain_range_.begin(), domain_range_.end(),
            [](auto& lhs, auto& rhs) { return lhs.first < rhs.first; });

  // And precompute the slope
  slope_.resize(range_node.size() - 1);
  MSG_DEBUG("Interpolation Table (%s)", name_.c_str());
  MSG_DEBUG("row: domain, range, slope ");
  size_t i;
  for (i = 0; i < domain_range_.size() - 1; i++) {
    double dy = domain_range_[i + 1].second - domain_range_[i].second;
    double dx = domain_range_[i + 1].first - domain_range_[i].first;
    if (fabs(dx) < 1e-16) {
      ERROR("Interpolation domain entries are too close (or repeated)");
      return false;
    }
    slope_[i] = dy / dx;

    MSG_DEBUG("%zu: %lf, %lf, %lf", i, domain_range_[i].first,
              domain_range_[i].second, slope_[i]);
  }
  MSG_DEBUG("%zu: %lf, %lf, N/A", i, domain_range_[i].first,
            domain_range_[i].second);

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for LinearInterpolation Device");
    return false;
  }

  return true;
}

bool fastcat::LinearInterpolation::Read()
{
  // update input signal
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }
  double signal_value = signals_[0].value;

  double domain_min = domain_range_.front().first;
  double domain_max = domain_range_.back().first;

  double range_min = domain_range_.front().second;
  double range_max = domain_range_.back().second;

  state_->linear_interpolation_state.is_saturated = false;

  // Check if the input signal is off the interp table
  if (signal_value < domain_min) {
    state_->linear_interpolation_state.output       = range_min;
    state_->linear_interpolation_state.is_saturated = true;

  } else if (signal_value > domain_max) {
    state_->linear_interpolation_state.output       = range_max;
    state_->linear_interpolation_state.is_saturated = true;
  }

  if (state_->linear_interpolation_state.is_saturated) {
    // only return false to indicate a NEW fault condition
    if (enable_out_of_bounds_fault_ and (not device_fault_active_)) {
      ERROR(
          "Linear Interpolation (%s) out of interpolation domain (%g to %g) "
          "signal was (%g)",
          name_.c_str(), domain_min, domain_max, signal_value);
      return false;
    }

    // otherwise we do not fault, and just saturate
    return true;
  }

  //// calc linear interpolation: output[i] = y[i] + m[i](input[i] - x[i])
  // Find where in the domain we fall and compute linear interpolation
  // Performance improvement: could precompute slope array
  // Performance improvement: divide-and-conquer to find the pivot
  //   rather than linear scan for log(n)-time, rather than n-time here
  size_t i;
  for (i = 0; i < domain_range_.size() - 1; i++) {
    double x1 = domain_range_[i].first;
    double y1 = domain_range_[i].second;
    double x2 = domain_range_[i + 1].first;
    double y2 = domain_range_[i + 1].second;

    if ((signal_value <= x2) && (signal_value > x1)) {
      state_->linear_interpolation_state.output =
          y1 + (y2 - y1) / (x2 - x1) * (signal_value - x1);
      return true;
    }
  }

  ERROR("Linear Interpolation (%s) internal error for signal value (%g)",
        name_.c_str(), signal_value);
  return false;
}
