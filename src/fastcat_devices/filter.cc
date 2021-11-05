// Include related header (for cc files)
#include "fastcat/fastcat_devices/filter.h"

// Include c then c++ libraries
#include <string.h>

#include <cassert>
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::MovingAverageFilter::MovingAverageFilter(uint16_t buffer_size)
    : buffer_size_(buffer_size)
{
  assert(buffer_size > 0);
  buffer_.resize(buffer_size);
}

double fastcat::MovingAverageFilter::ApplyFilter(double new_data)
{
  assert(buffer_size_ > 0);

  double sum = 0;

  buffer_index_ = buffer_index_ % buffer_size_;  // never goes out of range
  buffer_[buffer_index_] = new_data;
  buffer_index_++;

  for (int i = 0; i < buffer_size_; ++i) {
    sum += buffer_[i];
  }

  return (sum / (double)buffer_size_);
}

fastcat::DigitalABFilter::DigitalABFilter(std::vector<double> A,
                                          std::vector<double> B)
    : A_(A), B_(B)
{
  assert(A.size() > 0);
  assert(B.size() > 0);

  input_buffer_.resize(B_.size());
  output_buffer_.resize(A_.size());
  MSG_DEBUG("Initialized A, M=%lu", A_.size());
  MSG_DEBUG("Initialized B, N=%lu", B_.size());
}

double fastcat::DigitalABFilter::ApplyFilter(double new_data)
{
  double value = 0;

  // Reshuffle buffers (TODO improve efficiency)
  for (size_t i = 0; i < input_buffer_.size() - 1; ++i) {
    int ind            = input_buffer_.size() - 1 - i;
    input_buffer_[ind] = input_buffer_[ind - 1];
  }
  input_buffer_[0] = new_data;

  for (size_t i = 0; i < output_buffer_.size() - 1; ++i) {
    int ind             = output_buffer_.size() - 1 - i;
    output_buffer_[ind] = output_buffer_[ind - 1];
  }

  // Input and B terms
  for (size_t k = 0; k < B_.size(); ++k) {
    value += B_[k] * input_buffer_[k];
  }

  // Output and A terms
  for (size_t k = 1; k < A_.size(); ++k) {
    value -= A_[k] * output_buffer_[k];
  }

  output_buffer_[0] = value;

  return value;
}

fastcat::FilterType fastcat::FilterTypeFromString(std::string str)
{
  FilterType type = BAD_FILTER_TYPE;

  if (str.compare("MOVING_AVERAGE") == 0) {
    type = MOVING_AVERAGE;
  } else if (str.compare("DIGITAL_AB") == 0) {
    type = DIGITAL_AB;
  } else {
    ERROR("%s is not a known FilterType", str.c_str());
  }

  return type;
}

fastcat::Filter::Filter()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = FILTER_STATE;
}

bool fastcat::Filter::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "filter_type", filter_type_string_)) {
    return false;
  }

  filter_type_ = FilterTypeFromString(filter_type_string_);
  if (filter_type_ == BAD_FILTER_TYPE) {
    return false;
  }

  YAML::Node          coeff_node;
  std::vector<double> A_coeffs;
  std::vector<double> B_coeffs;
  switch (filter_type_) {
    case MOVING_AVERAGE:
      int buf_size;
      if (!ParseVal(node, "buffer_size", buf_size)) {
        return false;
      }

      mov_avg_ = std::make_unique<MovingAverageFilter>(buf_size);
      break;

    case DIGITAL_AB:

      if (!ParseList(node, "A", coeff_node)) {
        return false;
      }
      for (auto coeff = coeff_node.begin(); coeff != coeff_node.end();
           ++coeff) {
        A_coeffs.push_back((*coeff).as<double>());
      }

      if (!ParseList(node, "B", coeff_node)) {
        return false;
      }
      for (auto coeff = coeff_node.begin(); coeff != coeff_node.end();
           ++coeff) {
        B_coeffs.push_back((*coeff).as<double>());
      }

      digital_ab_ = std::make_unique<DigitalABFilter>(A_coeffs, B_coeffs);
      break;

    default:
      ERROR("Bad filter type");
      return false;
      break;
  }

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for Filter");
    return false;
  }

  return true;
}

bool fastcat::Filter::Read()
{
  if (!UpdateSignal(signals_[0])) {
    return false;
  }

  switch (filter_type_) {
    case MOVING_AVERAGE:
      state_->filter_state.output = mov_avg_->ApplyFilter(signals_[0].value);
      break;
    case DIGITAL_AB:
      state_->filter_state.output = digital_ab_->ApplyFilter(signals_[0].value);
      ;
      break;
    default:
      ERROR("Unhandled filter type");
      return false;
  }
  return true;
}
