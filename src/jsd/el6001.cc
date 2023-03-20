// Include related header (for cc files)
#include "fastcat/jsd/el6001.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::El6001::El6001()
{
  MSG_DEBUG("Constructed El6001");

  state_       = std::make_shared<DeviceState>();
  state_->type = EL6001_STATE;
}

bool fastcat::El6001::ConfigFromYaml(YAML::Node node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::El6001::ConfigFromYamlCommon(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EL6001_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  return true;
}

bool fastcat::El6001::Read()
{
  jsd_el6001_read((jsd_t*)context_, slave_id_);

  const jsd_el6001_state_t* jsd_state =
      jsd_el6001_get_state((jsd_t*)context_, slave_id_);

  state_->el6001_state.statusword   = jsd_state->statusword;
  state_->el6001_state.controlword   = jsd_state->controlword_user;  

  return true;
}
