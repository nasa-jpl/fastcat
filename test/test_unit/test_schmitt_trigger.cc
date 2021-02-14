#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/schmitt_trigger.h"
#include "fastcat/signal_handling.h"

namespace
{
class SchmittTriggerTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    std::string unit_test_dir(FASTCAT_UNIT_TEST_DIR);
    unit_test_dir += "test_schmitt_trigger_yamls/";
    config_node = YAML::LoadFile(unit_test_dir + "st1_config.yaml");
    st2_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "st2_config.yaml"));
    st3_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "st3_config.yaml"));
  }

  YAML::Node              config_node;
  fastcat::SchmittTrigger st1_;
  fastcat::SchmittTrigger st2_;
  fastcat::SchmittTrigger st3_;
};

TEST_F(SchmittTriggerTest, ConfigFromYamlSuccess)
{
  // ConfigFromYaml() returns true if all necessary fields are present in the
  // YAML file
  EXPECT_TRUE(st1_.ConfigFromYaml(config_node));
}

TEST_F(SchmittTriggerTest, ConfigFromYamlIncorrectFieldType)
{
  // ConfigFromYaml() throws an exception if a necessary field contains data
  // that cannot be converted to the appropriate type
  config_node["low_threshold"] = "JPL";
  EXPECT_ANY_THROW(st1_.ConfigFromYaml(config_node));
}

TEST_F(SchmittTriggerTest, ConfigFromYamlMissingField)
{
  // ConfigFromYaml() returns false if a necessary field is missing from the
  // YAML file
  config_node.remove("low_threshold");
  EXPECT_FALSE(st1_.ConfigFromYaml(config_node));
}

TEST_F(SchmittTriggerTest, ConfigFromYamlMissingSignalField)
{
  // ConfigFromYaml() returns false if any signal node is missing a necessary
  // field
  config_node["signals"][0].remove("request_signal_name");
  EXPECT_FALSE(st1_.ConfigFromYaml(config_node));
}

TEST_F(SchmittTriggerTest, ConfigFromYamlIncorrectSignals)
{
  // ConfigFromYaml() returns false if the YAML file provides the wrong number
  // of signals
  YAML::Node second_sig;
  second_sig["observed_device_name"] = "sig_gen_2";
  second_sig["request_signal_name"]  = "output";
  config_node["signals"].push_back(second_sig);
  EXPECT_FALSE(st1_.ConfigFromYaml(config_node));
}

TEST_F(SchmittTriggerTest, ReadResponse)
{
  // Read() returns false when signal cannot be updated
  EXPECT_FALSE(st2_.Read());

  // Read() returns true when signal updates successfully
  fastcat::DeviceState sgs;
  sgs.type                          = fastcat::SIGNAL_GENERATOR_STATE;
  sgs.signal_generator_state.output = 10.0;
  fastcat::ConfigSignalByteIndexing(&sgs, st2_.signals_[0]);

  EXPECT_TRUE(st2_.Read());

  // Read() returns true when signal is FIXED_VALUE
  EXPECT_TRUE(st3_.Read());
}

TEST_F(SchmittTriggerTest, WriteResponse)
{
  // Write() returns false because SchmittTrigger devices do not accept any
  // commands
  fastcat::DeviceCmd cmd;
  EXPECT_FALSE(st2_.Write(cmd));
}

TEST_F(SchmittTriggerTest, ProcessResponse)
{
  // Process() always returns NO_FAULT
  EXPECT_EQ(st2_.Process(), fastcat::NO_FAULT);
}
}  // namespace
