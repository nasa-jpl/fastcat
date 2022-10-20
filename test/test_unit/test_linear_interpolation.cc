#include <gtest/gtest.h>
#include <cmath>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/linear_interpolation.h"
#include "fastcat/signal_handling.h"

#include "jsd/jsd_print.h"

namespace
{
class LinearInterpolationTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    base_dir_ = FASTCAT_UNIT_TEST_DIR;
    base_dir_ += "test_linear_interpolation_yamls/";
  }

  std::string base_dir_;
  YAML::Node node_;
  fastcat::LinearInterpolation device_;
};

TEST_F(LinearInterpolationTest, InvalidNoName) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_1.yaml")));
}

TEST_F(LinearInterpolationTest, InvalidNoOutOfBoundsFlag) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_2.yaml")));
}

TEST_F(LinearInterpolationTest, InvalidSizeMismatch) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_3.yaml")));
}

TEST_F(LinearInterpolationTest, InvalidTableTooSmall) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_4.yaml")));
}

TEST_F(LinearInterpolationTest, InvalidRepeated) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_5.yaml")));
}

TEST_F(LinearInterpolationTest, InvalidManySignals) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_6.yaml")));
}

TEST_F(LinearInterpolationTest, InvalidNoSignals) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"invalid_7.yaml")));
}

TEST_F(LinearInterpolationTest, ValidAbs) {
  EXPECT_TRUE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"valid_abs.yaml")));
  // This table is valid from -30 to +20 and should be a perfect fabs function 
  // in this range.

  auto state = device_.GetState();
  double *output = &state->linear_interpolation_state.output;
  double *input = &device_.signals_[0].value;

  *input = -100;
  EXPECT_FALSE(device_.Read()); // off the interp table
  EXPECT_NEAR(*output, 30, 1e-12);
  device_.Reset();

  *input = 100;
  EXPECT_FALSE(device_.Read()); // off the interp table
  EXPECT_NEAR(*output, 20, 1e-12);
  device_.Reset();


  // Test nominal ranges
  double step = 1.5;
  *input = -15;
  while(*input < 15){
    *input += step;
    EXPECT_TRUE(device_.Read()); 
    EXPECT_NEAR(*output, fabs(*input), 1e-12);
    MSG("testing fabs(%lf) = %lf ", *input, *output);
  }

  // finer-grained
  step = 0.001;
  *input = -30;
  while(*input < (20 - step)){
    *input += step;
    EXPECT_TRUE(device_.Read()); 
    EXPECT_NEAR(*output, fabs(*input), 1e-12);
  }
}


TEST_F(LinearInterpolationTest, ValidAbsDecadeTests) {
  EXPECT_TRUE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"valid_abs_no_error.yaml")));
  // This table is valid from -30 to +20 and should be a perfect fabs function 
  // in this range.

  auto state = device_.GetState();
  double *output = &state->linear_interpolation_state.output;
  double *input = &device_.signals_[0].value;

  // the val here increases logarithimically like so:
  //      v--- start here since (i=-4) 
  // 0.0001
  // 0.0002
  // ...
  // 0.0009
  // 0.0010
  // 0.0020
  // ...
  // 0.0090
  // 0.0100
  for(int i = -6; i < 0; i++){
    double decade = pow(10, i);
    for(int j = 1; j < 10; j++){
      double val = (double)j * decade;
      MSG_DEBUG("val: %0.6lf", val);

      // off table, low-end
      *input = -30 - val;
      EXPECT_TRUE(device_.Read()); 
      EXPECT_NEAR(*output, 30, 1e-12);

      *input = -30 + val;
      EXPECT_TRUE(device_.Read()); 
      EXPECT_NEAR(*output, fabs(*input), 1e-12);

      // At the zero pivot
      *input = 0 - val;
      EXPECT_TRUE(device_.Read()); 
      EXPECT_NEAR(*output, fabs(*input), 1e-12);

      *input = 0 + val;
      EXPECT_TRUE(device_.Read()); 
      EXPECT_NEAR(*output, fabs(*input), 1e-12);

      // off table, high-end
      *input = 20 + val;
      EXPECT_TRUE(device_.Read()); 
      EXPECT_NEAR(*output, 20, 1e-12);

      *input = 20 - val;
      EXPECT_TRUE(device_.Read()); 
      EXPECT_NEAR(*output, fabs(*input), 1e-12);
    }
  }

}

}  // namespace
