#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/function.h"
#include "fastcat/signal_handling.h"

namespace
{
class FunctionTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    std::string unit_test_dir(FASTCAT_UNIT_TEST_DIR);
    unit_test_dir += "test_function_yamls/";
    config_node = YAML::LoadFile(unit_test_dir + "f1_config.yaml");
    f2_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "f2_config.yaml"));
    f3_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "f3_config.yaml"));
  }

  YAML::Node        config_node;
  fastcat::Function f1_;
  fastcat::Function f2_;
  fastcat::Function f3_;
};

TEST_F(FunctionTest, ConfigFromYamlSuccess)
{
  // ConfigFromYaml() returns true if all necessary fields are present in the
  // YAML file
  EXPECT_TRUE(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ConfigFromYamlBadFunctionType)
{
  // ConfigFromYaml() returns false if function_type is not POLYNOMIAL
  config_node["function_type"] = "NOT_GOOD";
  EXPECT_FALSE(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ConfigFromYamlIncorrectFieldType)
{
  // ConfigFromYaml() throws an exception if a necessary field contains data
  // that cannot be converted to the appropriate type
  config_node["order"] = "JPL";
  EXPECT_ANY_THROW(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ConfigFromYamlMissingField)
{
  // ConfigFromYaml() returns false if a necessary field is missing from the
  // YAML file
  config_node.remove("order");
  EXPECT_FALSE(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ConfigFromYamlIncorrectOrder)
{
  // ConfigFromYaml() returns false if the number of coefficients does not match
  // the order
  config_node["order"] = 3;
  EXPECT_FALSE(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ConfigFromYamlMissingSignalField)
{
  // ConfigFromYaml() returns false if any signal node is missing a necessary
  // field
  config_node["signals"][0].remove("request_signal_name");
  EXPECT_FALSE(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ConfigFromYamlIncorrectSignals)
{
  // ConfigFromYaml() returns false if the YAML file provides the wrong number
  // of signals
  YAML::Node second_sig;
  second_sig["observed_device_name"] = "sig_gen_2";
  second_sig["request_signal_name"]  = "output";
  config_node["signals"].push_back(second_sig);
  EXPECT_FALSE(f1_.ConfigFromYaml(config_node));
}

TEST_F(FunctionTest, ReadResponse)
{
  // Read() returns false when signal cannot be updated
  EXPECT_FALSE(f2_.Read());

  // Read() returns true when signal updates successfully for POLYNOMIAL
  // Functions
  fastcat::DeviceState sgs;
  sgs.type                          = fastcat::SIGNAL_GENERATOR_DEVICE;
  sgs.signal_generator_state.output = 10.0;
  fastcat::ConfigSignalByteIndexing(&sgs, f2_.signals_[0]);

  EXPECT_TRUE(f2_.Read());

  // Read() returns true when signal is FIXED_VALUE
  EXPECT_TRUE(f3_.Read());

  // Error will occur during ConfigFromYaml, so BAD_FUNCTION_TYPE case in Read()
  // cannot be reached
}

TEST_F(FunctionTest, WriteResponse)
{
  // Write() returns false because Function devices do not accept any commands
  fastcat::DeviceCmd cmd;
  EXPECT_FALSE(f2_.Write(cmd));
}

TEST_F(FunctionTest, ProcessResponse)
{
  // Process() always returns NO_FAULT
  EXPECT_EQ(f2_.Process(), fastcat::NO_FAULT);
}
}  // namespace
