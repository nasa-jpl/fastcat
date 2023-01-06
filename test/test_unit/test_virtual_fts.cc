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
    // FASTCAT_UNIT_TEST_DIR contains path to .
    base_dir_ = FASTCAT_UNIT_TEST_DIR;
    base_dir_ += "test_virtual_fts_yamls/";
  }

  std::string base_dir_;
  YAML::Node node_;
  fastcat::VirtualFts device_;
};

TEST_F(VirtualFtsTest, ParseNominalConfig) {
  EXPECT_TRUE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"nominal.yaml")));
}
