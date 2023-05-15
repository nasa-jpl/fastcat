#include <gtest/gtest.h>

#include <cmath>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/fts.h"
#include "fastcat/signal_handling.h"
#include "jsd/jsd_print.h"

class FtsTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    // FASTCAT_UNIT_TEST_DIR contains path to .
    base_dir_ = FASTCAT_UNIT_TEST_DIR;
    base_dir_ += "test_fts_yamls/";
  }

  std::string  base_dir_;
  YAML::Node   node_;
  fastcat::Fts device_;
};

TEST_F(FtsTest, ParseNominalConfig)
{
  EXPECT_TRUE(
      device_.ConfigFromYaml(YAML::LoadFile(base_dir_ + "nominal.yaml")));
}

// Currently the nominal behavior to faults is to prevent taring when faulted
// This may change with optional YAML parameters in the future perhaps.
TEST_F(FtsTest, RejectTareWhenFaulted)
{
  EXPECT_TRUE(
      device_.ConfigFromYaml(YAML::LoadFile(base_dir_ + "nominal.yaml")));

  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::FTS_TARE_CMD;
  EXPECT_TRUE(device_.Write(cmd));
  MSG("Accepted tare when in nominal state");

  device_.Fault();

  cmd.type = fastcat::FTS_TARE_CMD;
  EXPECT_FALSE(device_.Write(cmd));
}

TEST_F(FtsTest, WideMatrixValid)
{
  EXPECT_TRUE(
      device_.ConfigFromYaml(YAML::LoadFile(base_dir_ + "fts_wide_cal_matrix.yaml")));

  // Zero out all signals
  std::vector<fastcat::DeviceState> device_states(device_.signals_.size());
  for (int i=0; i<(int)device_.signals_.size(); i++) 
  { 
    auto &sgs = device_states[i]; 
    sgs.type = fastcat::SIGNAL_GENERATOR_STATE;
    sgs.signal_generator_state.output = 0.0;
    fastcat::ConfigSignalByteIndexing(&sgs, device_.signals_[i]);
  }
  device_.Read();
  EXPECT_EQ(
    0.0, device_.GetState()->fts_state.raw_tz);

  // Make non-zero final signal
  auto &last_state = device_states.back();
  last_state.signal_generator_state.output = 35.0;
  device_.Read();
  EXPECT_EQ(
    35.0, device_.GetState()->fts_state.raw_tz);
}