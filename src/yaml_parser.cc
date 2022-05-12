// Include related header (for cc files)
#include "fastcat/yaml_parser.h"

// Include c then c++ libraries

// Include external then project includes
#include "jsd/jsd_print.h"

bool fastcat::ParseNode(YAML::Node node, std::string field, YAML::Node& val)
{
  if (!node[field]) {
    ERROR("Expecting YAML Node: %s", field.c_str());
    return false;
  }
  val = node[field];
  MSG_DEBUG("Parsed Node: %s", field.c_str());
  return true;
}

bool fastcat::ParseList(YAML::Node node, std::string field, YAML::Node& val)
{
  if (!node[field]) {
    ERROR("Expecting YAML List Node: %s", field.c_str());
    return false;
  }
  if (!node[field].IsSequence()) {
    ERROR("Expecting %s to be a sequence", field.c_str());
    return false;
  }
  val = node[field];
  MSG_DEBUG("Parsed List Node: %s", field.c_str());
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, double& val)
{
  if (!node[field]) {
    ERROR("Expecting double field: %s", field.c_str());
    return false;
  }
  val = node[field].as<double>();
  MSG_DEBUG("Parsed double field %s: %lf", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, float& val)
{
  if (!node[field]) {
    ERROR("Expecting float field: %s", field.c_str());
    return false;
  }
  val = node[field].as<float>();
  MSG_DEBUG("Parsed float field %s: %f", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, std::string& val)
{
  if (!node[field]) {
    ERROR("Expecting string field: %s", field.c_str());
    return false;
  }
  val = node[field].as<std::string>();
  MSG_DEBUG("Parsed string field %s: %s", field.c_str(), val.c_str());
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, int32_t& val)
{
  if (!node[field]) {
    ERROR("Expection int32_t field: %s", field.c_str());
    return false;
  }
  val = node[field].as<int32_t>();
  MSG_DEBUG("Parsed int32_t field %s: %i", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, uint32_t& val)
{
  if (!node[field]) {
    ERROR("Expection uint32_t field: %s", field.c_str());
    return false;
  }
  val = node[field].as<uint32_t>();
  MSG_DEBUG("Parsed uint32_t field %s: %i", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, int16_t& val)
{
  if (!node[field]) {
    ERROR("Expection int16_t field: %s", field.c_str());
    return false;
  }
  val = node[field].as<int16_t>();
  MSG_DEBUG("Parsed int16_t field %s: %i", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, uint16_t& val)
{
  if (!node[field]) {
    ERROR("Expection uint16_t field: %s", field.c_str());
    return false;
  }
  val = node[field].as<uint16_t>();
  MSG_DEBUG("Parsed uint16_t field %s: %i", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, int8_t& val)
{
  if (!node[field]) {
    ERROR("Expection int8_t field: %s", field.c_str());
    return false;
  }
  val = node[field].as<int8_t>();
  MSG_DEBUG("Parsed int8_t field %s: %i", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, uint8_t& val)
{
  if (!node[field]) {
    ERROR("Expection uint8_t field: %s", field.c_str());
    return false;
  }
  val = node[field].as<uint8_t>();
  MSG_DEBUG("Parsed uint8_t field %s: %i", field.c_str(), val);
  return true;
}

bool fastcat::ParseVal(YAML::Node node, std::string field, bool& val)
{
  if (!node[field]) {
    ERROR("Expection bool field: %s", field.c_str());
    return false;
  }
  val = node[field].as<bool>();
  MSG_DEBUG("Parsed bool field %s: %s", field.c_str(),
            val > 0 ? "true" : "false");
  return true;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 double& val, double lower, double upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %lf <= %lf <= %lf", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field, float& val,
                                 float lower, float upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %f <= %f <= %f", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 int32_t& val, int32_t lower, int32_t upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %i <= %i <= %i", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 uint32_t& val, uint32_t lower, uint32_t upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %u <= %u <= %u", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 int16_t& val, int16_t lower, int16_t upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %i <= %i <= %i", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 uint16_t& val, int16_t lower, int16_t upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %u <= %u <= %u", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 int8_t& val, int8_t lower, int8_t upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %i <= %i <= %i", field.c_str(), lower, val,
        upper);
  return false;
}

bool fastcat::ParseValCheckRange(YAML::Node node, std::string field,
                                 uint8_t& val, uint8_t lower, uint8_t upper)
{
  if (!ParseVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %u <= %u <= %u", field.c_str(), lower, val,
        upper);
  return false;
}

// Optional value parsing

bool fastcat::ParseOptVal(YAML::Node node, std::string field, double& val)
{
  if (!node[field]) {
    MSG_DEBUG("Did not find optional double field: %s", field.c_str());
    return false;
  }
  val = node[field].as<double>();
  MSG_DEBUG("Parsed double field %s: %lf", field.c_str(), val);
  return true;
}

bool fastcat::ParseOptVal(YAML::Node node, std::string field, std::string& val)
{
  if (!node[field]) {
    MSG_DEBUG("Did not find optional string field: %s", field.c_str());
    return false;
  }
  val = node[field].as<std::string>();
  MSG_DEBUG("Parsed string field %s: %s", field.c_str(), val.c_str());
  return true;
}

bool fastcat::ParseOptValCheckRange(YAML::Node node, std::string field,
                                    double& val, double lower, double upper)
{
  if (!ParseOptVal(node, field, val)) {
    return false;
  }

  if (lower <= val && val <= upper) {
    return true;
  }
  ERROR("%s failed range check %lf <= %lf <= %lf", field.c_str(), lower, val,
        upper);
  return false;
}
