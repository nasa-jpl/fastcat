#ifndef FASTCAT_YAML_PARSER_H_
#define FASTCAT_YAML_PARSER_H_

// Include related header (for cc files)

// Include c then c++ libraries
#include <string>
#include <vector>

// Include external then project includes
#include <yaml-cpp/yaml.h>

namespace fastcat
{
bool ParseNode(YAML::Node node, std::string field, YAML::Node& val);
bool ParseList(YAML::Node node, std::string field, YAML::Node& val);

bool ParseVal(YAML::Node node, std::string field, double& val);
bool ParseVal(YAML::Node node, std::string field, float& val);
bool ParseVal(YAML::Node node, std::string field, std::string& val);
bool ParseVal(YAML::Node node, std::string field, int32_t& val);
bool ParseVal(YAML::Node node, std::string field, uint32_t& val);
bool ParseVal(YAML::Node node, std::string field, int16_t& val);
bool ParseVal(YAML::Node node, std::string field, uint16_t& val);
bool ParseVal(YAML::Node node, std::string field, int8_t& val);
bool ParseVal(YAML::Node node, std::string field, uint8_t& val);
bool ParseVal(YAML::Node node, std::string field, bool& val);

bool ParseValCheckRange(YAML::Node node, std::string field, double& val,
                        double lower, double upper);

bool ParseValCheckRange(YAML::Node node, std::string field, float& val,
                        float lower, float upper);

bool ParseValCheckRange(YAML::Node node, std::string field, int32_t& val,
                        int32_t lower, int32_t upper);

bool ParseValCheckRange(YAML::Node node, std::string field, uint32_t& val,
                        uint32_t lower, uint32_t upper);

bool ParseValCheckRange(YAML::Node node, std::string field, int16_t& val,
                        int16_t lower, int16_t upper);

bool ParseValCheckRange(YAML::Node node, std::string field, uint16_t& val,
                        int16_t lower, int16_t upper);

bool ParseValCheckRange(YAML::Node node, std::string field, int8_t& val,
                        int8_t lower, int8_t upper);

bool ParseValCheckRange(YAML::Node node, std::string field, uint8_t& val,
                        uint8_t lower, uint8_t upper);

// Optional double values
bool ParseOptVal(YAML::Node node, std::string field, double& val);
bool ParseOptVal(YAML::Node node, std::string field, std::string& val);
bool ParseOptVal(YAML::Node node, std::string field, bool& val);

bool ParseOptValCheckRange(YAML::Node node, std::string field, double& val,
                           double lower, double upper);

}  // namespace fastcat

#endif
