#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/yaml_parser.h"

namespace
{
class YamlParserTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    std::string unit_test_dir(FASTCAT_TEST_DIR);
    unit_test_dir += "device_tests/test_yaml_parser_yamls/";
    parsable_node = YAML::LoadFile(
        unit_test_dir + "yaml_parser_config.yaml");
  }
  YAML::Node  parsable_node;
  YAML::Node  empty_node;
  YAML::Node  val_node;
  YAML::Node  list_node;
  double      double_val;
  float       float_val;
  std::string string_val;
  int32_t     int32_val;
  uint8_t     uint8_val;
  uint32_t    uint32_val;
  bool        bool_val;
};

TEST_F(YamlParserTest, ParseNode)
{
  // ParseNode() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseNode(empty_node, "node_ex", val_node));
  EXPECT_TRUE(val_node.IsNull());

  // ParseNode() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseNode(parsable_node, "node_ex", val_node));
  EXPECT_TRUE(val_node.IsDefined());
  EXPECT_EQ(val_node, parsable_node["node_ex"]);
}

TEST_F(YamlParserTest, ParseList)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseList(empty_node, "list_ex", list_node));
  EXPECT_TRUE(list_node.IsNull());

  // ParseList() returns false if the desired field is not a sequence
  EXPECT_FALSE(fastcat::ParseList(parsable_node, "double_ex", list_node));
  EXPECT_TRUE(list_node.IsNull());

  // ParseList() returns true if the desired list field is found
  EXPECT_TRUE(fastcat::ParseList(parsable_node, "list_ex", list_node));
  EXPECT_TRUE(list_node.IsSequence());
  EXPECT_EQ(list_node, parsable_node["list_ex"]);
}

TEST_F(YamlParserTest, ParseDoubleVal)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "double_ex", double_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "double_ex", double_val));
  EXPECT_EQ(double_val, parsable_node["double_ex"].as<double>());
}

TEST_F(YamlParserTest, ParseFloatVal)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "float_ex", float_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "float_ex", float_val));
  EXPECT_EQ(float_val, parsable_node["float_ex"].as<float>());
}

TEST_F(YamlParserTest, ParseStringVal)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "string_ex", string_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "string_ex", string_val));
  EXPECT_EQ(string_val, parsable_node["string_ex"].as<std::string>());
}

TEST_F(YamlParserTest, ParseInt32Val)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "int32_ex", int32_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "int32_ex", int32_val));
  EXPECT_EQ(int32_val, parsable_node["int32_ex"].as<int32_t>());
}

TEST_F(YamlParserTest, ParseUInt8Val)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "uint8_ex", uint8_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "uint8_ex", uint8_val));
  EXPECT_EQ(uint8_val, parsable_node["uint8_ex"].as<uint8_t>());
}

TEST_F(YamlParserTest, ParseUInt32Val)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "uint32_ex", uint32_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "uint32_ex", uint32_val));
  EXPECT_EQ(uint32_val, parsable_node["uint32_ex"].as<uint32_t>());
}

TEST_F(YamlParserTest, ParseBoolVal)
{
  // ParseList() returns false if the desired field is not found in the node
  EXPECT_FALSE(fastcat::ParseVal(empty_node, "bool_ex", bool_val));

  // ParseList() returns true if the desired field is found
  EXPECT_TRUE(fastcat::ParseVal(parsable_node, "bool_ex", bool_val));
  EXPECT_EQ(bool_val, parsable_node["bool_ex"].as<bool>());
}
}  // namespace
