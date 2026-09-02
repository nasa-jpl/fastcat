#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "fastcat/fastcat.h"
#include "profile_utils.h"

namespace
{
// Extra time beyond the computed profile duration to stay in the RUN phase, so
// the drive has slack to settle on target before we halt it.
constexpr double kRunMarginS = 1.0;

void PrintUsage(const char* program)
{
  std::cerr << "Usage: " << program
            << " <config_path> <actuator_name> <accel> <max_velocity> "
               "<relative_position>"
            << std::endl;
  std::cerr << "  config_path: relative path to fastcat YAML config"
            << std::endl;
  std::cerr << "  actuator_name: name of actuator in YAML" << std::endl;
  std::cerr << "  accel: acceleration/deceleration (rad/s²)" << std::endl;
  std::cerr << "  max_velocity: maximum velocity (rad/s)" << std::endl;
  std::cerr << "  relative_position: position change (rad, can be negative)"
            << std::endl;
}

}  // namespace

int main(int argc, char** argv)
{
  profile_utils::PosProfileArgs args;
  std::string                   error;
  if (!profile_utils::ParsePosProfileArgs(
          std::vector<std::string>(argv + 1, argv + argc), args, error)) {
    std::cerr << "Error: " << error << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }

  YAML::Node node;
  try {
    node = YAML::LoadFile(args.config_path);
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading YAML file '" << args.config_path
              << "': " << e.what() << std::endl;
    return 1;
  }

  if (!profile_utils::ValidateActuatorName(node, args.actuator_name)) {
    return 1;
  }

  profile_utils::InstallSignalHandlers();

  fastcat::Manager mgr;
  std::cout << "Initializing fastcat manager..." << std::endl;
  if (!mgr.ConfigFromYaml(node)) {
    std::cerr << "Error: Failed to configure manager from YAML" << std::endl;
    return 1;
  }

  double loop_rate = mgr.GetTargetLoopRate();
  std::cout << "Loop rate: " << loop_rate << " Hz" << std::endl;
  if (loop_rate < profile_utils::kMinRecommendedLoopRateHz) {
    std::cerr << "Warning: Loop rate " << loop_rate
              << " Hz is below minimum recommended "
              << profile_utils::kMinRecommendedLoopRateHz << " Hz" << std::endl;
  }

  if (!profile_utils::RunResetPhase(mgr, args.actuator_name, loop_rate)) {
    mgr.Shutdown();
    return 1;
  }

  std::string   csv_filename = profile_utils::MakeTelemetryFilename("pos_prof");
  std::ofstream csv_file(csv_filename);
  if (!csv_file.is_open()) {
    std::cerr << "Error: Failed to open CSV file: " << csv_filename
              << std::endl;
    mgr.Shutdown();
    return 1;
  }

  csv_file << "unix_time_s,relative_time_s,cmd_position_rad,actual_position_"
              "rad,actual_velocity_rad_s,actual_current_A"
           << std::endl;
  std::cout << "Writing telemetry to: " << csv_filename << std::endl;

  double distance = std::abs(args.relative_position);
  double expected_duration = profile_utils::TrapezoidalMoveDuration(
      distance, args.accel, args.max_velocity);

  // Resolved in ISSUE against the measured position, then read back by
  // on_run_complete to report the residual position error.
  double target_position = 0.0;

  profile_utils::ProfileHooks hooks;
  hooks.run_ticks = profile_utils::TicksFromSeconds(
      expected_duration + kRunMarginS, loop_rate);

  hooks.issue = [&](const fastcat::GoldActuatorState& act_state) {
    double initial_position = act_state.actual_position;
    target_position         = initial_position + args.relative_position;

    fastcat::DeviceCmd cmd;
    cmd.type                                = fastcat::ACTUATOR_PROF_POS_CMD;
    cmd.actuator_prof_pos_cmd.target_position = target_position;
    cmd.actuator_prof_pos_cmd.profile_velocity = args.max_velocity;
    cmd.actuator_prof_pos_cmd.profile_accel    = args.accel;
    // Absolute, since we resolved the target against the measured position
    cmd.actuator_prof_pos_cmd.relative = 0;

    std::cout << "Profile position command issued" << std::endl;
    std::cout << "  Initial position: " << initial_position << " rad"
              << std::endl;
    std::cout << "  Target position: " << target_position << " rad"
              << std::endl;
    std::cout << "  Relative move: " << args.relative_position << " rad"
              << std::endl;
    return cmd;
  };

  hooks.write_csv_row = [&](const fastcat::GoldActuatorState& act_state,
                            double unix_time_s, double elapsed_s) {
    csv_file << std::fixed << std::setprecision(6) << unix_time_s << ","
             << elapsed_s << "," << act_state.cmd_position << ","
             << act_state.actual_position << "," << act_state.actual_velocity
             << "," << act_state.actual_current << std::endl;
  };

  hooks.log_run_tick = [](const fastcat::GoldActuatorState& act_state,
                          double elapsed_s) {
    std::cout << "  t=" << std::fixed << std::setprecision(2) << elapsed_s
              << "s  pos=" << std::setprecision(3) << act_state.actual_position
              << " rad  vel=" << act_state.actual_velocity
              << " rad/s  current=" << act_state.actual_current << " A"
              << std::endl;
  };

  hooks.on_run_complete = [&](const fastcat::GoldActuatorState& act_state) {
    std::cout << "  Final position: " << act_state.actual_position << " rad"
              << std::endl;
    std::cout << "  Position error: "
              << (target_position - act_state.actual_position) << " rad"
              << std::endl;
  };

  std::cout << "\nStarting control loop at " << loop_rate << " Hz..."
            << std::endl;
  std::cout << "Profile parameters: accel=" << args.accel
            << " rad/s², max_vel=" << args.max_velocity
            << " rad/s, distance=" << distance << " rad" << std::endl;
  std::cout << "Expected motion duration: " << expected_duration << " s"
            << std::endl;

  profile_utils::ProfileResult result =
      profile_utils::RunProfileLoop(mgr, args.actuator_name, loop_rate, hooks);

  csv_file.close();

  profile_utils::HaltAndSettle(mgr, args.actuator_name, loop_rate);

  std::cout << "\nShutting down..." << std::endl;
  mgr.Shutdown();

  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << "Commanded move:       " << std::fixed << std::setprecision(3)
            << args.relative_position << " rad" << std::endl;
  std::cout << "Peak cmd velocity:    " << result.peak_cmd_velocity << " rad/s"
            << std::endl;
  std::cout << "Peak actual velocity: " << result.peak_actual_velocity
            << " rad/s (noisy at low encoder resolution)" << std::endl;
  std::cout << "Telemetry saved to: " << csv_filename << std::endl;

  return 0;
}
