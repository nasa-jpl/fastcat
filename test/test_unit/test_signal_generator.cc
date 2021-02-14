#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/fastcat_devices/signal_generator.h"

namespace
{
class SignalGeneratorTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    std::string unit_test_dir(FASTCAT_UNIT_TEST_DIR);
    unit_test_dir += "test_signal_generator_yamls/";
    sin_config_node = YAML::LoadFile(unit_test_dir + "sg1_config.yaml");
    saw_config_node = YAML::LoadFile(unit_test_dir + "sg2_config.yaml");
    sg3_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "sg3_config.yaml"));
    sg4_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "sg4_config.yaml"));
  }

  YAML::Node               sin_config_node;
  YAML::Node               saw_config_node;
  fastcat::SignalGenerator sg1_;
  fastcat::SignalGenerator sg2_;
  fastcat::SignalGenerator sg3_;
  fastcat::SignalGenerator sg4_;
};

TEST_F(SignalGeneratorTest, ConfigFromYamlSuccess)
{
  // ConfigFromYaml() returns true if all necessary fields are present in the
  // YAML file
  EXPECT_TRUE(sg1_.ConfigFromYaml(sin_config_node));
  EXPECT_TRUE(sg2_.ConfigFromYaml(saw_config_node));
}

TEST_F(SignalGeneratorTest, ConfigFromYamlBadFunctionType)
{
  // ConfigFromYaml() returns false if signal_generator_type is not SINE_WAVE or
  // SAW_TOOTH
  saw_config_node["signal_generator_type"] = "NOT_GOOD";
  EXPECT_FALSE(sg2_.ConfigFromYaml(saw_config_node));
}

TEST_F(SignalGeneratorTest, ConfigFromYamlIncorrectFieldType)
{
  // ConfigFromYaml() throws an exception if a necessary field contains data
  // that cannot be converted to the appropriate type
  sin_config_node["angular_frequency"] = "JPL";
  EXPECT_ANY_THROW(sg1_.ConfigFromYaml(sin_config_node));
}

TEST_F(SignalGeneratorTest, ConfigFromYamlMissingField)
{
  // ConfigFromYaml() returns false if a necessary field is missing from the
  // YAML file
  sin_config_node.remove("angular_frequency");
  EXPECT_FALSE(sg1_.ConfigFromYaml(sin_config_node));

  saw_config_node.remove("slope");
  EXPECT_FALSE(sg2_.ConfigFromYaml(saw_config_node));
}

TEST_F(SignalGeneratorTest, ReadResponse)
{
  // Read() returns true for SINE_WAVE SignalGenerators
  EXPECT_TRUE(sg3_.Read());

  // Read() returns true for SAW_TOOTH SignalGenerators
  EXPECT_TRUE(sg4_.Read());

  // Error will occur during ConfigFromYaml, so default case in Read() cannot be
  // reached
}

TEST_F(SignalGeneratorTest, WriteResponse)
{
  // Write() returns false because SignalGenerator devices do not accept any
  // commands
  fastcat::DeviceCmd cmd;
  EXPECT_FALSE(sg3_.Write(cmd));
  EXPECT_FALSE(sg4_.Write(cmd));
}

TEST_F(SignalGeneratorTest, ProcessResponse)
{
  // Process() always returns NO_FAULT
  EXPECT_EQ(sg3_.Process(), fastcat::NO_FAULT);
  EXPECT_EQ(sg4_.Process(), fastcat::NO_FAULT);
}
}  // namespace
