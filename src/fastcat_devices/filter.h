#ifndef FASTCAT_FILTER_H_
#define FASTCAT_FILTER_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
class MovingAverageFilter
{
 public:
  MovingAverageFilter(uint16_t buffer_size);
  double ApplyFilter(double new_data);

 private:
  std::vector<double> buffer_;
  uint16_t            buffer_size_  = 0;
  uint16_t            buffer_index_ = 0;
};

class DigitalABFilter
{
 public:
  DigitalABFilter(std::vector<double> A, std::vector<double> B);
  double ApplyFilter(double new_data);

 private:
  std::vector<double> input_buffer_;
  std::vector<double> output_buffer_;
  std::vector<double> A_;
  std::vector<double> B_;
};

enum FilterType { MOVING_AVERAGE, DIGITAL_AB, BAD_FILTER_TYPE };

FilterType FilterTypeFromString(std::string str);

class Filter : public DeviceBase
{
 public:
  Filter();
  bool ConfigFromYaml(const YAML::Node& node, double external_time = -1) override;
  bool Read() override;

 protected:
  std::string                          filter_type_string_;
  enum FilterType                      filter_type_ = BAD_FILTER_TYPE;
  std::unique_ptr<MovingAverageFilter> mov_avg_;
  std::unique_ptr<DigitalABFilter>     digital_ab_;
};

}  // namespace fastcat

#endif
