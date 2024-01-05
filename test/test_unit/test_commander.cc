#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/thread_safe_queue.h"
#include "fastcat/fastcat_devices/commander.h"

namespace
{
class CommanderTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    std::string unit_test_dir(FASTCAT_UNIT_TEST_DIR);
    unit_test_dir += "test_commander_yamls/";
    config_node = YAML::LoadFile(unit_test_dir + "c1_config.yaml");
    c2_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "c2_config.yaml"));
    c3_.ConfigFromYaml(YAML::LoadFile(unit_test_dir + "c3_config.yaml"));

    cmd_queue_ = std::make_shared<fastcat::ThreadSafeQueue<fastcat::DeviceCmd>>();
    c2_.RegisterCmdQueue(cmd_queue_);
    c3_.RegisterCmdQueue(cmd_queue_);
  }

  YAML::Node                                      config_node;
  fastcat::Commander                              c1_;
  fastcat::Commander                              c2_;
  fastcat::Commander                              c3_;
  std::shared_ptr<fastcat::ThreadSafeQueue<fastcat::DeviceCmd>> cmd_queue_;
};

TEST_F(CommanderTest, ConfigFromYamlSuccess)
{
  // ConfigFromYaml() returns true if all necessary fields are present
  EXPECT_TRUE(c1_.ConfigFromYaml(config_node));
}

TEST_F(CommanderTest, ConfigFromYamlBadCommandType)
{
  // ConfigFromYaml() returns false if device_cmd_type is invalid
  config_node["device_cmd_type"] = "NOT_GOOD";
  EXPECT_FALSE(c1_.ConfigFromYaml(config_node));
}

TEST_F(CommanderTest, ConfigFromYamlMissingField)
{
  // ConfigFromYaml() returns false if a necessary field is missing from the
  // YAML file
  config_node.remove("start_enabled");
  EXPECT_FALSE(c1_.ConfigFromYaml(config_node));
}

TEST_F(CommanderTest, ReadResponseFixedSignals)
{
  // Read() returns true when all signals are FIXED_VALUE
  EXPECT_TRUE(c3_.Read());
}

TEST_F(CommanderTest, WriteValidCommand)
{
  // Write() returns true if the command sent is valid for Commander devices
  fastcat::DeviceCmd cmd;
  cmd.type                          = fastcat::COMMANDER_ENABLE_CMD;
  cmd.commander_enable_cmd.duration = 30;
  EXPECT_TRUE(c2_.Write(cmd));
  EXPECT_TRUE(c2_.GetState()->commander_state.enable);
}

TEST_F(CommanderTest, WriteInvalidCommand)
{
  // Write() returns false if the command sent is not valid for Commander
  // devices
  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::BAD_DEVICE_CMD;
  EXPECT_FALSE(c2_.Write(cmd));
}

TEST_F(CommanderTest, ProcessResponse)
{
  // Process() always returns NO_FAULT
  EXPECT_EQ(c2_.Process(), fastcat::NO_FAULT);
}

TEST_F(CommanderTest, CmdsRejectedWhenFaulted)
{
  // Put in faulted state
  c2_.Fault();

  // Reject Disable
  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::COMMANDER_DISABLE_CMD;

  EXPECT_FALSE(c2_.Write(cmd));
  EXPECT_FALSE(c2_.GetState()->commander_state.enable);

  // Reject Enable
  cmd.type                          = fastcat::COMMANDER_ENABLE_CMD;
  cmd.commander_enable_cmd.duration = 30;

  EXPECT_FALSE(c2_.Write(cmd));
  EXPECT_FALSE(c2_.GetState()->commander_state.enable);

  // Reset, then accept Enable
  c2_.Reset();
  EXPECT_TRUE(c2_.Write(cmd));
  EXPECT_TRUE(c2_.GetState()->commander_state.enable);
}

}  // namespace
