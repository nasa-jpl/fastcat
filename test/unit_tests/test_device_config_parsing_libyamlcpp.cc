#include <gtest/gtest.h>
#include "fastcat/device_config_parsing_libyamlcpp.h"
#include "fastcat/config.h"

TEST(device_config_parsing_libyamlcpp, ConditionalOperatorType){
  
  std::string unit_test_dir(FASTCAT_TEST_DIR);
  unit_test_dir += "device_tests/test_conditional_yamls/";
  auto config_node = YAML::LoadFile(unit_test_dir + "c1_config.yaml");

  EXPECT_EQ(config_node["conditional_type"].as<fastcat::ConditionalOperatorType>(), fastcat::GT);

  auto cfg = config_node.as<fastcat::ConditionalConfig>();
  EXPECT_EQ(cfg.conditional_type, fastcat::GT);
  EXPECT_DOUBLE_EQ(cfg.compare_rhs_value, 100.0);

}
