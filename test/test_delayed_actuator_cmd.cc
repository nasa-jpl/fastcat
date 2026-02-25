#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
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
  std::cerr << "Usage: " << program_name
            << " <process_loop_frequency_hz> <actuator_name> <delay_sec> "
               "<target_position> <profile_velocity> <profile_accel> "
               "<fastcat_yaml_config_path>\n";
}
}  // namespace

int main(int argc, char* argv[])
{
  if (argc != 8) {
    PrintUsage(argv[0]);
    return 1;
  }

  double process_loop_frequency_hz = 0.0;
  double delay_sec                 = 0.0;
  double target_position           = 0.0;
  double profile_velocity          = 0.0;
  double profile_accel             = 0.0;
  try {
    process_loop_frequency_hz = std::stod(argv[1]);
    delay_sec                 = std::stod(argv[3]);
    target_position           = std::stod(argv[4]);
    profile_velocity          = std::stod(argv[5]);
    profile_accel             = std::stod(argv[6]);
  } catch (const std::exception&) {
    std::cerr << "Invalid numeric argument. "
                 "Frequency, delay, target_position, profile_velocity, and "
                 "profile_accel must be valid numbers.\n";
    PrintUsage(argv[0]);
    return 1;
  }

  if (process_loop_frequency_hz <= 0.0) {
    std::cerr << "Frequency must be > 0.\n";
    return 1;
  }
  if (delay_sec < 0.0) {
    std::cerr << "Delay must be >= 0.\n";
    return 1;
  }

  std::string actuator_name(argv[2]);
  std::string yaml_config_path(argv[7]);

  fastcat::Manager fcat_manager;

  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  YAML::Node node;
  try {
    node = YAML::LoadFile(yaml_config_path);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load YAML file '" << yaml_config_path
              << "': " << e.what() << "\n";
    return 1;
  }

  if (!fcat_manager.ConfigFromYaml(node)) {
    std::cerr << "Could not configure fastcat manager from YAML.\n";
    return 1;
  }

  std::vector<std::string> gold_actuator_names;
  fcat_manager.GetDeviceNamesByType(gold_actuator_names,
                                    fastcat::GOLD_ACTUATOR_STATE);

  std::cout << "Gold actuators found (" << gold_actuator_names.size() << "):\n";
  for (const auto& name : gold_actuator_names) {
    std::cout << "  " << name << "\n";
  }

  const auto loop_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / process_loop_frequency_hz));
  const auto start_time = std::chrono::steady_clock::now();
  auto       next_time  = start_time;
  bool       sent_cmd   = false;

  while (!g_should_exit.load()) {
    if (!fcat_manager.Process()) {
      std::cerr << "fastcat manager faulted. Exiting process loop.\n";
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
        cmd.name                                     = actuator_name;
        cmd.type                                     = fastcat::ACTUATOR_PROF_POS_CMD;
        cmd.actuator_prof_pos_cmd.target_position   = target_position;
        cmd.actuator_prof_pos_cmd.profile_velocity  = profile_velocity;
        cmd.actuator_prof_pos_cmd.profile_accel     = profile_accel;
        cmd.actuator_prof_pos_cmd.relative          = 1;
        fcat_manager.QueueCommand(cmd);

        std::cout << "Queued ACTUATOR_PROF_POS_CMD for device '" << actuator_name
                  << "' at t=" << elapsed_sec
                  << " sec with target_position=" << target_position
                  << ", profile_velocity=" << profile_velocity
                  << ", profile_accel=" << profile_accel << "\n";
        sent_cmd = true;
      }
    }

    next_time += loop_period;
    std::this_thread::sleep_until(next_time);
  }

  fcat_manager.Shutdown();
  return 0;
}
