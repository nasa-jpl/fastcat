#include <gtest/gtest.h>

#include <cmath>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/three_node_thermal_model.h"
#include "fastcat/signal_handling.h"
#include "jsd/jsd_print.h"

static constexpr double DOUBLE_COMP_THRESHOLD = 1e-10;

class ThreeNodeThermalModelTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    base_dir_ = FASTCAT_UNIT_TEST_DIR;
    base_dir_ += "test_three_node_thermal_model_yamls/";
    device_.SetLoopPeriod(1.0);
  }

  std::string                    base_dir_;
  YAML::Node                     node_;
  fastcat::ThreeNodeThermalModel device_;
};

TEST_F(ThreeNodeThermalModelTest, SetupValid)
{
  node_ = YAML::LoadFile(base_dir_ + "valid.yaml");
  EXPECT_TRUE(device_.ConfigFromYaml(node_));
}

TEST_F(ThreeNodeThermalModelTest, BadConfigNumSignals)
{
  node_ = YAML::LoadFile(base_dir_ + "invalid_num_signals.yaml");
  EXPECT_FALSE(device_.ConfigFromYaml(node_));
}

TEST_F(ThreeNodeThermalModelTest, BadConfigNumTemps)
{
  node_ = YAML::LoadFile(base_dir_ + "invalid_num_temps.yaml");
  EXPECT_FALSE(device_.ConfigFromYaml(node_));
}

TEST_F(ThreeNodeThermalModelTest, SeedTemp)
{  // init device
  node_ = YAML::LoadFile(base_dir_ + "valid.yaml");
  EXPECT_TRUE(device_.ConfigFromYaml(node_));
  // check initial state outputs
  auto state = device_.GetState();

  // inputs
  double* node_3_temp_input   = &device_.signals_[0].value;
  double* motor_current_input = &device_.signals_[1].value;

  // outputs
  double* node_1_temp_output =
      &state->three_node_thermal_model_state.node_1_temp;
  double* node_2_temp_output =
      &state->three_node_thermal_model_state.node_2_temp;
  double* node_3_temp_output =
      &state->three_node_thermal_model_state.node_3_temp;
  double* node_4_temp_output =
      &state->three_node_thermal_model_state.node_4_temp;

  EXPECT_NEAR(*node_1_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);

  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::SEED_THERMAL_MODEL_TEMPERATURE_CMD;
  cmd.seed_thermal_model_temperature_cmd.seed_temperature = 20.0;
  EXPECT_TRUE(device_.Write(cmd));
  *node_3_temp_input   = 20.0;
  *motor_current_input = 0.0;
  EXPECT_TRUE(device_.Read());
  EXPECT_EQ(device_.Process(), fastcat::NO_FAULT);
  EXPECT_NEAR(*node_1_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
}

TEST_F(ThreeNodeThermalModelTest, CurrentInput)
{
  // init device
  node_ = YAML::LoadFile(base_dir_ + "valid.yaml");
  EXPECT_TRUE(device_.ConfigFromYaml(node_));
  // check initial state outputs
  auto state = device_.GetState();

  // inputs
  double* node_3_temp_input   = &device_.signals_[0].value;
  double* motor_current_input = &device_.signals_[1].value;

  // outputs
  double* node_1_temp_output =
      &state->three_node_thermal_model_state.node_1_temp;
  double* node_2_temp_output =
      &state->three_node_thermal_model_state.node_2_temp;
  double* node_3_temp_output =
      &state->three_node_thermal_model_state.node_3_temp;
  double* node_4_temp_output =
      &state->three_node_thermal_model_state.node_4_temp;

  EXPECT_NEAR(*node_1_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);

  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::SEED_THERMAL_MODEL_TEMPERATURE_CMD;
  cmd.seed_thermal_model_temperature_cmd.seed_temperature = 20.0;
  EXPECT_TRUE(device_.Write(cmd));
  *node_3_temp_input   = 20.0;
  *motor_current_input = 1.0;
  EXPECT_TRUE(device_.Read());
  device_.SetTime(1.0, 1.0);
  EXPECT_EQ(device_.Process(), fastcat::NO_FAULT);
  EXPECT_NEAR(*node_1_temp_output, 21.25, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 20.416666666666668, DOUBLE_COMP_THRESHOLD);
  device_.SetTime(2.0, 2.0);
  EXPECT_EQ(device_.Process(), fastcat::NO_FAULT);
  EXPECT_NEAR(*node_1_temp_output, 20.9375, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 21.25, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 20.729166666666668, DOUBLE_COMP_THRESHOLD);
  device_.SetTime(3.0, 3.0);
  EXPECT_EQ(device_.Process(), fastcat::NO_FAULT);
  EXPECT_NEAR(*node_1_temp_output, 22.578125, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 19.6875, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 20.755208333333332, DOUBLE_COMP_THRESHOLD);
  device_.SetTime(4.0, 4.0);
  EXPECT_EQ(device_.Process(), fastcat::NO_FAULT);
  EXPECT_NEAR(*node_1_temp_output, 20.21484375, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 22.890625, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 20.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 21.03515625, DOUBLE_COMP_THRESHOLD);
}

TEST_F(ThreeNodeThermalModelTest, TempFault)
{
  // init device
  node_ = YAML::LoadFile(base_dir_ + "valid.yaml");
  EXPECT_TRUE(device_.ConfigFromYaml(node_));
  // check initial state outputs
  auto state = device_.GetState();

  // inputs
  double* node_3_temp_input   = &device_.signals_[0].value;
  double* motor_current_input = &device_.signals_[1].value;

  // outputs
  double* node_1_temp_output =
      &state->three_node_thermal_model_state.node_1_temp;
  double* node_2_temp_output =
      &state->three_node_thermal_model_state.node_2_temp;
  double* node_3_temp_output =
      &state->three_node_thermal_model_state.node_3_temp;
  double* node_4_temp_output =
      &state->three_node_thermal_model_state.node_4_temp;

  EXPECT_NEAR(*node_1_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_2_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_3_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);
  EXPECT_NEAR(*node_4_temp_output, 0.0, DOUBLE_COMP_THRESHOLD);

  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::SEED_THERMAL_MODEL_TEMPERATURE_CMD;
  cmd.seed_thermal_model_temperature_cmd.seed_temperature = 20.0;
  EXPECT_TRUE(device_.Write(cmd));
  *node_3_temp_input   = 20.0;
  *motor_current_input = 100.0;
  EXPECT_TRUE(device_.Read());
  device_.SetTime(1.0, 1.0);
  EXPECT_EQ(device_.Process(), fastcat::ALL_DEVICE_FAULT);
}
