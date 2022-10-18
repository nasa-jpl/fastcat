#include <gtest/gtest.h>
#include <cmath>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/linear_interpolation.h"
#include "fastcat/signal_handling.h"

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

TEST_F(LinearInterpolationTest, ValidSquared) {
  EXPECT_FALSE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"valid_squared.yaml")));

  EXPECT_FALSE(device_.Read()); // off the interp table
  device_.Reset();

  device_.signals_[0].value = 10;
  EXPECT_FALSE(device_.Read()); // off the interp table
  device_.Reset();

  auto state = device_.GetState();

  device_.signals_[0].value = 1.5;
  EXPECT_TRUE(device_.Read()); 
  EXPECT_NEAR(state->linear_interpolation_state.output, pow(1.5, 2), 1.0);
  
  device_.signals_[0].value = 2.5;
  EXPECT_TRUE(device_.Read()); 
  EXPECT_NEAR(state->linear_interpolation_state.output, pow(2.5, 2), 1.0);

  device_.signals_[0].value = 3.5;
  EXPECT_TRUE(device_.Read()); 
  EXPECT_NEAR(state->linear_interpolation_state.output, pow(3.5, 2), 1.0);

  device_.signals_[0].value = 4.5;
  EXPECT_TRUE(device_.Read()); 
  EXPECT_NEAR(state->linear_interpolation_state.output, pow(4.5, 2), 1.0);


}


}  // namespace
