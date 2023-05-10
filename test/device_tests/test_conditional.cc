#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/conditional.h"
#include "fastcat/signal_handling.h"

namespace
{
class ConditionalTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    std::string unit_test_dir(FASTCAT_TEST_DIR);
    unit_test_dir += "device_tests/test_conditional_yamls/";
    config_node = YAML::LoadFile(unit_test_dir + "c1_config.yaml");
    c2_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "c2_config.yaml"));
    c3_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "c3_config.yaml"));
  }

  YAML::Node           config_node;
  fastcat::Conditional c1_;
  fastcat::Conditional c2_;
  fastcat::Conditional c3_;
};

TEST_F(ConditionalTest, ConfigFromYamlSuccess)
{
  // ConfigFromYaml() returns true if all necessary fields are present in the
  // YAML file
  EXPECT_TRUE(c1_.ConfigFromYaml(config_node));
}

TEST_F(ConditionalTest, ConfigFromYamlIncorrectFieldType)
{
  // ConfigFromYaml() throws an exception if a necessary field contains data
  // that cannot be converted to the appropriate type
  config_node["compare_rhs_value"] = "JPL";
  EXPECT_ANY_THROW(c1_.ConfigFromYaml(config_node));
}

TEST_F(ConditionalTest, ConfigFromYamlBadConditionalType)
{
  // ConfigFromYaml() returns false if conditional_type is not a valid option
  config_node["conditional_type"] = "is equal to";
  EXPECT_FALSE(c1_.ConfigFromYaml(config_node));
}

TEST_F(ConditionalTest, ConfigFromYamlMissingField)
{
  // ConfigFromYaml() returns false if a necessary field is missing from the
  // YAML file
  config_node.remove("compare_rhs_value");
  EXPECT_FALSE(c1_.ConfigFromYaml(config_node));
}

TEST_F(ConditionalTest, ConfigFromYamlMissingSignalField)
{
  // ConfigFromYaml() returns false if any signal node is missing a necessary
  // field
  config_node["signals"][0].remove("request_signal_name");
  EXPECT_FALSE(c1_.ConfigFromYaml(config_node));
}

TEST_F(ConditionalTest, ConfigFromYamlIncorrectSignals)
{
  // ConfigFromYaml() returns false if the YAML file provides the wrong number
  // of signals
  YAML::Node second_sig;
  second_sig["observed_device_name"] = "sig_gen_2";
  second_sig["request_signal_name"]  = "output";
  config_node["signals"].push_back(second_sig);
  EXPECT_FALSE(c1_.ConfigFromYaml(config_node));
}

TEST_F(ConditionalTest, ReadResponse)
{
  // Read() returns false when signal cannot be updated
  EXPECT_FALSE(c2_.Read());

  // Read() returns true when signal updates successfully
  fastcat::DeviceState sgs;
  sgs.type                          = fastcat::SIGNAL_GENERATOR_DEVICE;
  sgs.signal_generator_state.output = 10.0;
  fastcat::ConfigSignalByteIndexing(&sgs, c2_.signals_[0]);

  EXPECT_TRUE(c2_.Read());

  // Read() returns true when signal is FIXED_VALUE
  EXPECT_TRUE(c3_.Read());
}

TEST_F(ConditionalTest, WriteResponse)
{
  // Write() returns false because Conditional devices do not accept any
  // commands
  fastcat::DeviceCmd cmd;
  EXPECT_FALSE(c2_.Write(cmd));
}

TEST_F(ConditionalTest, ProcessResponse)
{
  // Process() always returns NO_FAULT
  EXPECT_EQ(c2_.Process(), fastcat::NO_FAULT);
}
}  // namespace
