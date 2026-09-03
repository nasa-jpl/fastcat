#include "profile_utils.h"

#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "jsd/jsd_elmo_common.h"

namespace profile_utils
{
std::atomic<bool> g_shutdown{false};

namespace
{
void SignalHandler(int signum)
{
  std::cout << "\nReceived signal " << signum << ", shutting down..."
            << std::endl;
  g_shutdown = true;
}

// "SWITCH_ON_DISABLED (0x40)"
std::string DescribeElmoState(uint32_t elmo_state_machine_state)
{
  std::ostringstream oss;
  oss << jsd_elmo_state_machine_state_to_string(
             static_cast<jsd_elmo_state_machine_state_t>(
                 elmo_state_machine_state))
      << " (0x" << std::hex << elmo_state_machine_state << std::dec << ")";
  return oss.str();
}

}  // namespace

void InstallSignalHandlers()
{
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
}

bool IsSafeElmoState(uint32_t elmo_state_machine_state)
{
  for (auto safe_state : kSafeElmoStates) {
    if (elmo_state_machine_state == static_cast<uint32_t>(safe_state)) {
      return true;
    }
  }
  return false;
}

int TicksFromSeconds(double seconds, double loop_rate_hz)
{
  return static_cast<int>(seconds * loop_rate_hz);
}

std::vector<std::string> CollectDeviceNames(const YAML::Node& node)
{
  std::vector<std::string> names;
  if (!node["buses"]) {
    return names;
  }
  for (const auto& bus : node["buses"]) {
    if (!bus["devices"]) {
      continue;
    }
    for (const auto& device : bus["devices"]) {
      if (device["name"]) {
        names.push_back(device["name"].as<std::string>());
      }
    }
  }
  return names;
}

bool ValidateActuatorName(const YAML::Node& node,
                          const std::string& actuator_name)
{
  std::vector<std::string> available_names = CollectDeviceNames(node);
  for (const auto& name : available_names) {
    if (name == actuator_name) {
      return true;
    }
  }

  std::cerr << "Error: Actuator '" << actuator_name
            << "' not found in YAML config." << std::endl;
  std::cerr << "Available actuator names: ";
  for (size_t i = 0; i < available_names.size(); ++i) {
    std::cerr << "'" << available_names[i] << "'";
    if (i < available_names.size() - 1) std::cerr << ", ";
  }
  std::cerr << std::endl;
  return false;
}

std::string MakeTelemetryFilename(const std::string& tag, std::time_t when)
{
  std::tm tm_when;
  localtime_r(&when, &tm_when);

  std::ostringstream oss;
  oss << std::put_time(&tm_when, "%Y%m%d_%H%M%S") << "_" << tag
      << "_telem.csv";
  return oss.str();
}

std::string MakeTelemetryFilename(const std::string& tag)
{
  return MakeTelemetryFilename(
      tag, std::chrono::system_clock::to_time_t(
               std::chrono::system_clock::now()));
}

double TrapezoidalMoveDuration(double distance, double accel,
                               double max_velocity)
{
  double accel_time     = max_velocity / accel;
  double accel_distance = 0.5 * accel * accel_time * accel_time;

  if (distance < 2.0 * accel_distance) {
    // Triangular profile - max_velocity is never reached
    return 2.0 * std::sqrt(distance / accel);
  }
  double cruise_distance = distance - 2.0 * accel_distance;
  return 2.0 * accel_time + cruise_distance / max_velocity;
}

double VelocityProfileDuration(double accel, double cruise_speed,
                               double cruise_duration)
{
  double ramp_time = std::abs(cruise_speed) / accel;
  return 2.0 * ramp_time + cruise_duration;
}

bool ParseDouble(const std::string& text, double& out)
{
  if (text.empty()) {
    return false;
  }
  try {
    size_t consumed = 0;
    double value    = std::stod(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    out = value;
    return true;
  } catch (const std::logic_error&) {
    return false;
  }
}

namespace
{
// Fills `out` from `args[index]`, or sets `error` naming the offending field.
bool ParseArg(const std::vector<std::string>& args, size_t index,
              const char* field, double& out, std::string& error)
{
  if (!ParseDouble(args[index], out)) {
    error = std::string(field) + ": '" + args[index] + "' is not a number";
    return false;
  }
  return true;
}

constexpr size_t kExpectedArgCount = 5;

bool CheckArgCount(const std::vector<std::string>& args, std::string& error)
{
  if (args.size() != kExpectedArgCount) {
    error = "expected " + std::to_string(kExpectedArgCount) +
            " arguments, got " + std::to_string(args.size());
    return false;
  }
  return true;
}

}  // namespace

bool ParseVelProfileArgs(const std::vector<std::string>& args,
                         VelProfileArgs& out, std::string& error)
{
  if (!CheckArgCount(args, error)) {
    return false;
  }

  out.config_path   = args[0];
  out.actuator_name = args[1];
  if (!ParseArg(args, 2, "accel", out.accel, error) ||
      !ParseArg(args, 3, "cruise_speed", out.cruise_speed, error) ||
      !ParseArg(args, 4, "cruise_duration", out.cruise_duration, error)) {
    return false;
  }

  if (out.accel <= 0) {
    error = "acceleration must be positive";
    return false;
  }
  if (out.cruise_speed == 0) {
    error = "cruise_speed must be non-zero";
    return false;
  }
  if (out.cruise_duration <= 0) {
    error = "cruise_duration must be positive";
    return false;
  }
  return true;
}

bool ParsePosProfileArgs(const std::vector<std::string>& args,
                         PosProfileArgs& out, std::string& error)
{
  if (!CheckArgCount(args, error)) {
    return false;
  }

  out.config_path   = args[0];
  out.actuator_name = args[1];
  if (!ParseArg(args, 2, "accel", out.accel, error) ||
      !ParseArg(args, 3, "max_velocity", out.max_velocity, error) ||
      !ParseArg(args, 4, "relative_position", out.relative_position, error)) {
    return false;
  }

  if (out.accel <= 0) {
    error = "acceleration must be positive";
    return false;
  }
  if (out.max_velocity <= 0) {
    error = "max_velocity must be positive";
    return false;
  }
  if (out.relative_position == 0) {
    error = "relative_position cannot be zero";
    return false;
  }
  return true;
}

const fastcat::GoldActuatorState* FindGoldActuatorState(
    const std::vector<fastcat::DeviceState>& states, const std::string& name)
{
  for (const auto& state : states) {
    if (state.name == name && state.type == fastcat::GOLD_ACTUATOR_STATE) {
      return &state.gold_actuator_state;
    }
  }
  return nullptr;
}

bool RunResetPhase(fastcat::Manager& mgr, const std::string& actuator_name,
                   double loop_rate_hz)
{
  const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz);

  std::cout << "Resetting actuator '" << actuator_name << "'..." << std::endl;

  fastcat::DeviceCmd reset_cmd;
  reset_cmd.name = actuator_name;
  reset_cmd.type = fastcat::ACTUATOR_RESET_CMD;
  mgr.QueueCommand(reset_cmd);

  auto     reset_start     = std::chrono::steady_clock::now();
  uint32_t last_elmo_state = kNoStateLogged;
  for (int i = 0; i < kResetTicks; ++i) {
    if (!mgr.Process()) {
      std::cerr << "Error: Manager process failed during reset phase at "
                   "iteration "
                << i << std::endl;
      std::cerr << "This likely indicates a hardware/communication issue with "
                   "the drive."
                << std::endl;
      std::cerr << "Check: 1) Motor is connected  2) Drive has power  3) "
                   "EtherCAT cable is secure"
                << std::endl;
      return false;
    }

    auto states = mgr.GetDeviceStates();
    auto act_state = FindGoldActuatorState(states, actuator_name);
    if (act_state && act_state->elmo_state_machine_state != last_elmo_state) {
      std::cout << "  [warmup tick " << i << "] elmo_sms="
                << DescribeElmoState(act_state->elmo_state_machine_state)
                << " act_sms=0x" << std::hex
                << act_state->actuator_state_machine_state << std::dec
                << " servo=" << (int)act_state->servo_enabled
                << " motor_on=" << (int)act_state->motor_on
                << " jsd_fault=" << act_state->jsd_fault_code << std::endl;
      last_elmo_state = act_state->elmo_state_machine_state;
    }
    std::this_thread::sleep_for(period);
  }

  double reset_duration = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - reset_start)
                              .count();
  std::cout << "Reset phase completed in " << std::fixed << std::setprecision(1)
            << reset_duration << "s, proceeding to warmup..." << std::endl;
  return true;
}

void HaltAndSettle(fastcat::Manager& mgr, const std::string& actuator_name,
                   double loop_rate_hz)
{
  const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz);
  const int  max_settle_ticks =
      TicksFromSeconds(kHaltSettleTimeoutS, loop_rate_hz);

  std::cout << "\nGraceful halt: waiting for drive to reach safe state..."
            << std::endl;
  {
    fastcat::DeviceCmd halt_cmd;
    halt_cmd.name = actuator_name;
    halt_cmd.type = fastcat::ACTUATOR_HALT_CMD;
    mgr.QueueCommand(halt_cmd);
  }

  uint32_t last_logged_sms  = kNoStateLogged;
  bool     reached_safe     = false;
  uint32_t final_sms        = 0;
  for (int i = 0; i < max_settle_ticks; ++i) {
    // Ignore Process() return - the drive may transit through fault states
    // (e.g. QUICK_STOP_ACTIVE) on the way down, and jsd_egd will auto-issue
    // FAULT_RESET to walk it back to SWITCH_ON_DISABLED.
    mgr.Process();

    auto states    = mgr.GetDeviceStates();
    auto act_state = FindGoldActuatorState(states, actuator_name);
    if (act_state) {
      final_sms = act_state->elmo_state_machine_state;
      if (final_sms != last_logged_sms) {
        std::cout << "  shutdown tick " << i << ": elmo_sms="
                  << DescribeElmoState(final_sms) << std::endl;
        last_logged_sms = final_sms;
      }
      reached_safe = IsSafeElmoState(final_sms);
    }
    if (reached_safe) break;
    std::this_thread::sleep_for(period);
  }

  if (reached_safe) {
    std::cout << "Drive reached safe state (elmo_sms="
              << DescribeElmoState(final_sms)
              << "); next run can start cleanly." << std::endl;
  } else {
    std::cerr << "Warning: drive did not reach a safe state within "
              << kHaltSettleTimeoutS << "s (last elmo_sms="
              << DescribeElmoState(final_sms) << ")." << std::endl;
    std::cerr << "  If next run reports 'off nominal', power-cycle the drive "
                 "first."
              << std::endl;
  }
}

ProfileResult RunProfileLoop(fastcat::Manager& mgr,
                             const std::string& actuator_name,
                             double loop_rate_hz, const ProfileHooks& hooks)
{
  enum Phase { ISSUE, BRAKE_DISENGAGE, RUN, RECOVER, DONE };

  const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz);
  const int  brake_disengage_ticks =
      TicksFromSeconds(kBrakeDisengageTimeoutS, loop_rate_hz);
  const int recover_settle_ticks =
      TicksFromSeconds(kRecoverSettleS, loop_rate_hz);

  Phase phase              = ISSUE;
  int   tick_count         = 0;
  int   run_tick_count     = 0;
  int   retry_count        = 0;
  int   recover_tick_count = 0;
  bool  motor_ready        = false;

  ProfileResult result;

  auto start_time = std::chrono::steady_clock::now();
  auto next_tick  = start_time + std::chrono::duration_cast<
                                    std::chrono::steady_clock::duration>(period);

  while (phase != DONE && !g_shutdown) {
    // Process the EtherCAT cycle. If a fault occurred and retries remain, drop
    // into RECOVER.
    if (!mgr.Process()) {
      if (phase == RUN) {
        std::cerr << "Error: Bus faulted during RUN phase" << std::endl;
        phase = DONE;
      } else if (phase != RECOVER && retry_count < kMaxRecoverRetries) {
        std::cerr << "Bus faulted during "
                  << (phase == ISSUE ? "ISSUE" : "BRAKE_DISENGAGE")
                  << " (attempt " << (retry_count + 1) << "/"
                  << kMaxRecoverRetries << "), entering RECOVER..."
                  << std::endl;
        phase              = RECOVER;
        recover_tick_count = 0;
      } else {
        std::cerr << "Error: Manager process failed and retries exhausted"
                  << std::endl;
        break;
      }
    }

    auto states    = mgr.GetDeviceStates();
    auto act_state = FindGoldActuatorState(states, actuator_name);
    if (!act_state) {
      std::cerr << "Error: Could not find actuator state for '" << actuator_name
                << "'" << std::endl;
      break;
    }

    double elapsed_time = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - start_time)
                              .count();
    double unix_time_s =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    hooks.write_csv_row(*act_state, unix_time_s, elapsed_time);

    if (std::abs(act_state->actual_velocity) > result.peak_actual_velocity) {
      result.peak_actual_velocity = std::abs(act_state->actual_velocity);
    }
    if (std::abs(act_state->cmd_velocity) > result.peak_cmd_velocity) {
      result.peak_cmd_velocity = std::abs(act_state->cmd_velocity);
    }

    switch (phase) {
      case ISSUE: {
        fastcat::DeviceCmd cmd = hooks.issue(*act_state);
        cmd.name               = actuator_name;
        mgr.QueueCommand(cmd);

        std::cout << "Waiting for brake disengagement and motor enable..."
                  << std::endl;
        phase      = BRAKE_DISENGAGE;
        tick_count = 0;
        break;
      }

      case BRAKE_DISENGAGE:
        if (!motor_ready && act_state->servo_enabled && act_state->motor_on) {
          std::cout << "Motor enabled (servo_enabled=1, motor_on=1)"
                    << std::endl;
          motor_ready = true;
          std::cout << "Executing motion profile..." << std::endl;
          phase          = RUN;
          run_tick_count = 0;
          break;
        }

        if (tick_count % kBrakeStatusPrintInterval == 0 && tick_count > 0) {
          std::cout << "  Waiting for motor enable: servo="
                    << (int)act_state->servo_enabled
                    << " motor_on=" << (int)act_state->motor_on << " state=0x"
                    << std::hex << act_state->actuator_state_machine_state
                    << std::dec << std::endl;
        }

        // Check for faults - the top-of-loop branch handles bus-level failures
        if (mgr.IsFaulted()) {
          std::cerr << "Manager faulted during brake disengagement"
                    << std::endl;
          std::cerr << "  Fault code: " << act_state->fastcat_fault_code
                    << "  EMCY: 0x" << std::hex << act_state->emcy_error_code
                    << std::dec << "  elmo_sms="
                    << DescribeElmoState(act_state->elmo_state_machine_state)
                    << std::endl;
          if (retry_count < kMaxRecoverRetries) {
            phase              = RECOVER;
            recover_tick_count = 0;
          } else {
            std::cerr << "Retries exhausted." << std::endl;
            phase = DONE;
          }
          break;
        }

        tick_count++;
        if (tick_count >= brake_disengage_ticks) {
          std::cerr << "Error: Motor did not enable within "
                    << kBrakeDisengageTimeoutS << "s" << std::endl;
          std::cerr << "servo_enabled=" << (int)act_state->servo_enabled
                    << " motor_on=" << (int)act_state->motor_on << " state=0x"
                    << std::hex << act_state->actuator_state_machine_state
                    << std::dec << std::endl;
          phase = DONE;
        }
        break;

      case RUN:
        // The trapezoid is self-driving once issued - do not re-queue the
        // motion command or we restart the profile every tick.
        if (run_tick_count % kRunStatusPrintInterval == 0) {
          hooks.log_run_tick(*act_state, elapsed_time);
        }

        run_tick_count++;
        if (run_tick_count >= hooks.run_ticks) {
          std::cout << "Motion profile complete" << std::endl;
          if (hooks.on_run_complete) {
            hooks.on_run_complete(*act_state);
          }
          phase = DONE;
        }

        if (mgr.IsFaulted()) {
          std::cerr << "Error: Manager reported fault" << std::endl;
          phase = DONE;
        }
        break;

      case RECOVER:
        if (recover_tick_count == 0) {
          std::cout << "RECOVER: calling ExecuteAllDeviceResets (retry "
                    << (retry_count + 1) << "/" << kMaxRecoverRetries << ")"
                    << std::endl;
          mgr.ExecuteAllDeviceResets();
        }
        recover_tick_count++;
        if (recover_tick_count >= recover_settle_ticks) {
          retry_count++;
          std::cout << "RECOVER: settle complete, reissuing motion command"
                    << "  elmo_sms="
                    << DescribeElmoState(act_state->elmo_state_machine_state)
                    << "  servo=" << (int)act_state->servo_enabled
                    << "  motor_on=" << (int)act_state->motor_on << std::endl;
          motor_ready = false;
          tick_count  = 0;
          phase       = ISSUE;
        }
        break;

      case DONE:
        break;
    }

    std::this_thread::sleep_until(next_tick);
    next_tick += std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
  }

  return result;
}

}  // namespace profile_utils
