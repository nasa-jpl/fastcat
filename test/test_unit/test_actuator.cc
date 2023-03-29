#include <gtest/gtest.h>

#include <cmath>

#include "fastcat/config.h"
#include "fastcat/jsd/actuator_offline.h"
#include "fastcat/signal_handling.h"
#include "jsd/jsd_print.h"

namespace
{
class ActuatorTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    jsd_context_ = jsd_alloc();

    // FASTCAT_UNIT_TEST_DIR contains path to .
    base_dir_ = FASTCAT_UNIT_TEST_DIR;
    base_dir_ += "test_actuator_yamls/";

    device_.SetSlaveId(0);
    device_.SetContext(jsd_context_);
    device_.SetOffline(true);
  }

  void TearDown() override { jsd_free(jsd_context_); }

  jsd_t*                   jsd_context_;
  std::string              base_dir_;
  YAML::Node               node_;
  fastcat::ActuatorOffline device_;
};

TEST_F(ActuatorTest, ParseValidWithPower)
{
  EXPECT_TRUE(device_.ConfigFromYaml(
      YAML::LoadFile(base_dir_ + "valid_with_power.yaml")));
}

TEST_F(ActuatorTest, ParseValidWithOptional)
{
  EXPECT_TRUE(device_.ConfigFromYaml(
      YAML::LoadFile(base_dir_ + "valid_with_opts.yaml")));
}

TEST_F(ActuatorTest, ParseValid)
{
  EXPECT_TRUE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_ + "valid.yaml")));
}

TEST_F(ActuatorTest, RejectMotionCommandsWhenFaulted)
{
  EXPECT_TRUE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_ + "valid.yaml")));

  device_.Fault();

  // Fail Motion commands
  {  // CSP
    fastcat::DeviceCmd cmd;
    cmd.type                                = fastcat::ACTUATOR_CSP_CMD;
    cmd.actuator_csp_cmd.target_position    = 0;
    cmd.actuator_csp_cmd.position_offset    = 0;
    cmd.actuator_csp_cmd.velocity_offset    = 0;
    cmd.actuator_csp_cmd.torque_offset_amps = 0;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // CSV
    fastcat::DeviceCmd cmd;
    cmd.type                                = fastcat::ACTUATOR_CSV_CMD;
    cmd.actuator_csv_cmd.target_velocity    = 0;
    cmd.actuator_csv_cmd.velocity_offset    = 0;
    cmd.actuator_csv_cmd.torque_offset_amps = 0;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // CST
    fastcat::DeviceCmd cmd;
    cmd.type                                = fastcat::ACTUATOR_CST_CMD;
    cmd.actuator_cst_cmd.target_torque_amps = 0;
    cmd.actuator_cst_cmd.torque_offset_amps = 0;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // Prof Pos
    fastcat::DeviceCmd cmd;
    cmd.type                                   = fastcat::ACTUATOR_PROF_POS_CMD;
    cmd.actuator_prof_pos_cmd.target_position  = 0;
    cmd.actuator_prof_pos_cmd.profile_velocity = 0.1;
    cmd.actuator_prof_pos_cmd.end_velocity     = 0;
    cmd.actuator_prof_pos_cmd.profile_accel    = 0.1;
    cmd.actuator_prof_pos_cmd.relative         = 0;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // Prof Vel
    fastcat::DeviceCmd cmd;
    cmd.type                                  = fastcat::ACTUATOR_PROF_VEL_CMD;
    cmd.actuator_prof_vel_cmd.target_velocity = 0.1;
    cmd.actuator_prof_vel_cmd.profile_accel   = 0.1;
    cmd.actuator_prof_vel_cmd.max_duration    = 30;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // Prof Torque
    fastcat::DeviceCmd cmd;
    cmd.type = fastcat::ACTUATOR_PROF_TORQUE_CMD;
    cmd.actuator_prof_torque_cmd.target_torque_amps = 0;
    cmd.actuator_prof_torque_cmd.max_duration       = 30;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // Calibrate
    fastcat::DeviceCmd cmd;
    cmd.type                               = fastcat::ACTUATOR_CALIBRATE_CMD;
    cmd.actuator_calibrate_cmd.velocity    = 0.02;
    cmd.actuator_calibrate_cmd.accel       = 0.2;
    cmd.actuator_calibrate_cmd.max_current = 0.2;
    EXPECT_FALSE(device_.Write(cmd));
  }
  {  // Halt
    fastcat::DeviceCmd cmd;

    cmd.type = fastcat::ACTUATOR_HALT_CMD;
    EXPECT_FALSE(device_.Write(cmd));
  }

  {  // Confirm non-motion commands are still accepted
    fastcat::DeviceCmd cmd;

    cmd.type = fastcat::ACTUATOR_SET_GAIN_SCHEDULING_INDEX_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SET_GAIN_SCHEDULING_INDEX_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SET_OUTPUT_POSITION_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SET_MAX_CURRENT_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SDO_SET_UNIT_MODE_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SDO_SET_UNIT_MODE_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SDO_DISABLE_GAIN_SCHEDULING_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SDO_ENABLE_SPEED_GAIN_SCHEDULING_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SDO_ENABLE_POSITION_GAIN_SCHEDULING_CMD;
    EXPECT_TRUE(device_.Write(cmd));

    cmd.type = fastcat::ACTUATOR_SDO_ENABLE_MANUAL_GAIN_SCHEDULING_CMD;
    EXPECT_TRUE(device_.Write(cmd));
  }
}

}  // namespace
