#include <gtest/gtest.h>
#include <cmath>

#include "fastcat/config.h"
#include "fastcat/jsd/ati_fts.h"
#include "fastcat/signal_handling.h"

#include "jsd/jsd.h"
#include "jsd/jsd_print.h"

namespace
{
class AtiFtsTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    jsd_context_ = jsd_alloc();

    // FASTCAT_UNIT_TEST_DIR contains path to .
    base_dir_ = FASTCAT_UNIT_TEST_DIR;
    base_dir_ += "test_ati_fts_yamls/";

    device_.SetSlaveId(0);
    device_.SetContext(jsd_context_);
    device_.SetOffline(true);
  }

  void TearDown() override
  {
    jsd_free(jsd_context_);
  }

  jsd_t* jsd_context_;
  std::string base_dir_;
  YAML::Node node_;
  fastcat::AtiFts device_;
};

TEST_F(AtiFtsTest, ParseNominalConfig) {
  EXPECT_TRUE(device_.ConfigFromYaml(YAML::LoadFile(base_dir_+"nominal.yaml")));
}

} // namespace
