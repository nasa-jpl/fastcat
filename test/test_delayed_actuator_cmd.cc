#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "fastcat/fastcat.h"

namespace
{
std::atomic<bool> g_should_exit(false);

void SignalHandler(int)
{
  g_should_exit.store(true);
}

void PrintUsage(const char* program_name)
{
  ERROR("Usage: %s"
        " <delay_sec> "
        "<target_position> <profile_velocity> <profile_accel> "
        "<fastcat_yaml_config_path>\n", program_name);
}
}  // namespace

int main(int argc, char* argv[])
{
  if (argc != 6) {
    PrintUsage(argv[0]);
    return 1;
  }

  double delay_sec        = 0.0;
  double target_position  = 0.0;
  double profile_velocity = 0.0;
  double profile_accel    = 0.0;
  try {
    delay_sec        = std::stod(argv[1]);
    target_position  = std::stod(argv[2]);
    profile_velocity = std::stod(argv[3]);
    profile_accel    = std::stod(argv[4]);
  } catch (const std::exception&) {
    ERROR("Invalid numeric argument. "
          "Delay, target_position, profile_velocity, and profile_accel "
          "must be valid numbers.");
    PrintUsage(argv[0]);
    return 1;
  }

  if (delay_sec < 0.0) {
    ERROR("Delay must be >= 0.");
    return 1;
  }

  std::string yaml_config_path(argv[5]);

  fastcat::Manager fcat_manager;

  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  YAML::Node node;
  try {
    node = YAML::LoadFile(yaml_config_path);
  } catch (const std::exception& e) {
    ERROR("Failed to load YAML file '%s': %s", yaml_config_path.c_str(), e.what());
    return 1;
  }

  if (!fcat_manager.ConfigFromYaml(node)) {
    ERROR("Could not configure fastcat manager from YAML.");
    return 1;
  }

  const double process_loop_frequency_hz = fcat_manager.GetTargetLoopRate();
  if (process_loop_frequency_hz <= 0.0) {
    ERROR("Invalid target_loop_rate_hz from YAML: %f", process_loop_frequency_hz);
    return 1;
  }

  std::vector<std::string> gold_actuator_names;
  fcat_manager.GetDeviceNamesByType(gold_actuator_names,
                                    fastcat::GOLD_ACTUATOR_STATE);

  if (gold_actuator_names.empty()) {
    ERROR("No GoldActuator found in topology.");
    return 1;
  }

  if (gold_actuator_names.size() > 1) {
    ERROR("Expected exactly one GoldActuator, found %zu.",
          gold_actuator_names.size());
    return 1;
  }

  const std::string& actuator_name = gold_actuator_names.front();

  MSG("Gold actuators found (%zu):", gold_actuator_names.size());
  for (const auto& name : gold_actuator_names) {
    MSG("%s", name.c_str());
  }

  const auto loop_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / process_loop_frequency_hz));
  const auto start_time = std::chrono::steady_clock::now();
  auto       next_time  = start_time;
  bool       sent_cmd   = false;

  while (!g_should_exit.load()) {
    if (!fcat_manager.Process()) {
      ERROR("fastcat manager faulted. Exiting process loop.");
      break;
    }

    if (!sent_cmd) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_sec =
          std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                     start_time)
              .count();

      if (elapsed_sec >= delay_sec) {
        fastcat::DeviceCmd cmd;
        cmd.name                                    = actuator_name;
        cmd.type                                    = fastcat::ACTUATOR_PROF_POS_CMD;
        cmd.actuator_prof_pos_cmd.target_position   = target_position;
        cmd.actuator_prof_pos_cmd.profile_velocity  = profile_velocity;
        cmd.actuator_prof_pos_cmd.profile_accel     = profile_accel;
        cmd.actuator_prof_pos_cmd.relative          = 1;
        fcat_manager.QueueCommand(cmd);

        ERROR("Queued ACTUATOR_PROF_POS_CMD for device '%s'"
              " at t=%f"
              " sec with target_position=%f"
              ", profile_velocity=%f"
              ", profile_accel=%f",
              actuator_name.c_str(), elapsed_sec, target_position,
              profile_velocity, profile_accel);
        sent_cmd = true;
      }
    }

    next_time += loop_period;
    std::this_thread::sleep_until(next_time);
  }

  fcat_manager.Shutdown();
  return 0;
}
