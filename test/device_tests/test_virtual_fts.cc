#include <gtest/gtest.h>

#include <cmath>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/virtual_fts.h"
#include "fastcat/signal_handling.h"
#include "jsd/jsd_print.h"

class VirtualFtsTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    base_dir_ = FASTCAT_TEST_DIR;
    base_dir_ += "device_tests/test_virtual_fts_yamls/";
  }

  std::string         base_dir_;
  YAML::Node          node_;
  fastcat::VirtualFts device_;
};

TEST_F(VirtualFtsTest, ParseNominalConfig)
{
  EXPECT_TRUE(
      device_.ConfigFromYaml(YAML::LoadFile(base_dir_ + "nominal.yaml")));
}

// Currently the nominal behavior to faults is to prevent taring when faulted
// This may change with optional YAML parameters in the future perhaps.
TEST_F(VirtualFtsTest, RejectTareWhenFaulted)
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
