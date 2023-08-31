// Include related header (for cc files)
#include "fastcat/jsd/ild1900.h"

// Include C then C++ libraries

// Include external then project includes
#include "fastcat/yaml_parser.h"

fastcat::Ild1900::Ild1900()
{
  MSG_DEBUG("Constructed ILD1900");

  state_       = std::make_shared<DeviceState>();
  state_->type = ILD1900_STATE;
}

bool fastcat::Ild1900::ConfigFromYaml(const YAML::Node& node)
{
  bool retval = ConfigFromYamlCommon(node);
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
  return retval;
}

bool fastcat::Ild1900::ConfigFromYamlCommon(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_ILD1900_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

  std::string model_string;
  if (!ParseVal(node, "model", model_string)) {
    return false;
  }
  if (!ModelFromString(model_string, jsd_slave_config_.ild1900.model)) {
    return false;
  }

  if (!ParseVal(node, "measuring_rate",
                jsd_slave_config_.ild1900.measuring_rate)) {
    return false;
  }

  std::string averaging_type_string;
  if (!ParseVal(node, "averaging_type", averaging_type_string)) {
    return false;
  }
  if (!AveragingTypeFromString(averaging_type_string,
                               jsd_slave_config_.ild1900.averaging_type)) {
    return false;
  }

  if (jsd_slave_config_.ild1900.averaging_type != JSD_ILD1900_AVERAGING_NONE) {
    if (!ParseVal(node, "averaging_number",
                  jsd_slave_config_.ild1900.averaging_number)) {
      return false;
    }
  }

  std::string exposure_mode_string;
  if (!ParseVal(node, "exposure_mode", exposure_mode_string)) {
    return false;
  }
  if (!ExposureModeFromString(exposure_mode_string,
                              jsd_slave_config_.ild1900.exposure_mode)) {
    return false;
  }

  std::string peak_selection_string;
  if (!ParseVal(node, "peak_selection", peak_selection_string)) {
    return false;
  }
  if (!PeakSelectionFromString(peak_selection_string,
                               jsd_slave_config_.ild1900.peak_selection)) {
    return false;
  }

  return true;
}

bool fastcat::Ild1900::Read()
{
  jsd_ild1900_read((jsd_t*)context_, slave_id_);

  const jsd_ild1900_state_t* jsd_state =
      jsd_ild1900_get_state((jsd_t*)context_, slave_id_);

  state_->ild1900_state.distance_m   = jsd_state->distance_m;
  state_->ild1900_state.intensity    = jsd_state->intensity;
  state_->ild1900_state.distance_raw = jsd_state->distance_raw;
  state_->ild1900_state.timestamp_us = jsd_state->timestamp_us;
  state_->ild1900_state.counter      = jsd_state->counter;
  state_->ild1900_state.error        = jsd_state->error;

  return true;
}

bool fastcat::Ild1900::ModelFromString(std::string          model_string,
                                       jsd_ild1900_model_t& model)
{
  MSG("Determining ILD1900's model from string.");
  if (model_string.compare("2") == 0) {
    model = JSD_ILD1900_MODEL_2;
  } else if (model_string.compare("10") == 0) {
    model = JSD_ILD1900_MODEL_10;
  } else if (model_string.compare("25") == 0) {
    model = JSD_ILD1900_MODEL_25;
  } else if (model_string.compare("50") == 0) {
    model = JSD_ILD1900_MODEL_50;
  } else if (model_string.compare("100") == 0) {
    model = JSD_ILD1900_MODEL_100;
  } else if (model_string.compare("200") == 0) {
    model = JSD_ILD1900_MODEL_200;
  } else if (model_string.compare("500") == 0) {
    model = JSD_ILD1900_MODEL_500;
  } else if (model_string.compare("2LL") == 0) {
    model = JSD_ILD1900_MODEL_2LL;
  } else if (model_string.compare("6LL") == 0) {
    model = JSD_ILD1900_MODEL_6LL;
  } else if (model_string.compare("10LL") == 0) {
    model = JSD_ILD1900_MODEL_10LL;
  } else if (model_string.compare("25LL") == 0) {
    model = JSD_ILD1900_MODEL_25LL;
  } else if (model_string.compare("50LL") == 0) {
    model = JSD_ILD1900_MODEL_50LL;
  } else {
    ERROR("ILD1900's model %s is invalid.", model_string.c_str());
    return false;
  }

  return true;
}

bool fastcat::Ild1900::AveragingTypeFromString(
    std::string averaging_type_string, jsd_ild1900_averaging_t& averaging_type)
{
  MSG("Determining ILD1900's measurement averaging type from string.");
  if (averaging_type_string.compare("NONE") == 0) {
    averaging_type = JSD_ILD1900_AVERAGING_NONE;
  } else if (averaging_type_string.compare("MEDIAN") == 0) {
    averaging_type = JSD_ILD1900_AVERAGING_MEDIAN;
  } else if (averaging_type_string.compare("MOVING") == 0) {
    averaging_type = JSD_ILD1900_AVERAGING_MOVING;
  } else if (averaging_type_string.compare("RECURSIVE") == 0) {
    averaging_type = JSD_ILD1900_AVERAGING_RECURSIVE;
  } else {
    ERROR("ILD1900's measurement averaging type %s is invalid.",
          averaging_type_string.c_str());
    return false;
  }

  return true;
}

bool fastcat::Ild1900::ExposureModeFromString(
    std::string                  exposure_mode_string,
    jsd_ild1900_exposure_mode_t& exposure_mode)
{
  MSG("Determining ILD1900's exposure mode from string.");
  if (exposure_mode_string.compare("STANDARD") == 0) {
    exposure_mode = JSD_ILD1900_EXPOSURE_MODE_STANDARD;
  } else if (exposure_mode_string.compare("INTELLIGENT") == 0) {
    exposure_mode = JSD_ILD1900_EXPOSURE_MODE_INTELLIGENT;
  } else if (exposure_mode_string.compare("BACKGROUND") == 0) {
    exposure_mode = JSD_ILD1900_EXPOSURE_MODE_BACKGROUND;
  } else {
    ERROR("ILD1900's exposure mode %s is invalid.",
          exposure_mode_string.c_str());
    return false;
  }

  return true;
}

bool fastcat::Ild1900::PeakSelectionFromString(
    std::string                   peak_selection_string,
    jsd_ild1900_peak_selection_t& peak_selection)
{
  MSG("Determining ILD's peak selection from string.");
  if (peak_selection_string.compare("HIGHEST") == 0) {
    peak_selection = JSD_ILD1900_PEAK_SELECTION_HIGHEST;
  } else if (peak_selection_string.compare("WIDEST") == 0) {
    peak_selection = JSD_ILD1900_PEAK_SELECTION_WIDEST;
  } else if (peak_selection_string.compare("LAST") == 0) {
    peak_selection = JSD_ILD1900_PEAK_SELECTION_LAST;
  } else if (peak_selection_string.compare("FIRST") == 0) {
    peak_selection = JSD_ILD1900_PEAK_SELECTION_FIRST;
  } else {
    ERROR("ILD1900's peak selection %s is invalid.",
          peak_selection_string.c_str());
    return false;
  }

  return true;
}