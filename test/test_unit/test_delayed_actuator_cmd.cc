#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>
#include <sys/timerfd.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <limits>

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
  ERROR("Usage: %s "
        "<delay_sec> "
        "<target_position> <profile_velocity> <profile_accel> "
        "<fastcat_yaml_config_path>\n", program_name);
}

std::chrono::nanoseconds ComputeLoopPeriod(double frequency_hz)
{
  const auto period_ns = static_cast<int64_t>(1e9 / frequency_hz);
  return std::chrono::nanoseconds(period_ns);
}

timespec ToTimespec(std::chrono::nanoseconds duration)
{
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  const auto rem_ns  = duration - seconds;

  timespec ts{};
  ts.tv_sec  = static_cast<time_t>(seconds.count());
  ts.tv_nsec = static_cast<long>(rem_ns.count());
  return ts;
}

// Monotonic time in seconds (double), similar spirit to this->now().seconds() but steady.
double NowMonotonicSeconds()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

void ProcessTimerThread(fastcat::Manager*              manager,
                        std::string                    actuator_name,
                        std::chrono::nanoseconds       loop_period,
                        std::atomic<bool>*             process_faulted)
{
  const int timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
  if (timer_fd < 0) {
    ERROR("Failed to create timerfd for Process thread (errno=%d).", errno);
    process_faulted->store(true);
    g_should_exit.store(true);
    return;
  }

  itimerspec timer_spec{};
  timer_spec.it_value    = ToTimespec(loop_period);
  timer_spec.it_interval = ToTimespec(loop_period);

  if (timerfd_settime(timer_fd, 0, &timer_spec, nullptr) != 0) {
    ERROR("Failed to configure timerfd for Process thread (errno=%d).", errno);
    close(timer_fd);
    process_faulted->store(true);
    g_should_exit.store(true);
    return;
  }

  // ---- Telemetry logging setup ----
  const double loop_period_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(loop_period).count();

  std::ofstream telemetry_csv("process_telemetry.csv",
                              std::ios::out | std::ios::trunc);
  if (!telemetry_csv.is_open()) {
    ERROR("Failed to open process_telemetry.csv for writing.");
    process_faulted->store(true);
    g_should_exit.store(true);
    close(timer_fd);
    return;
  }

  telemetry_csv << "t_sec,jitter_sec,expirations,position,velocity,current,power\n";
  telemetry_csv << std::fixed << std::setprecision(9);

  bool   have_last_time   = false;
  double last_time        = 0.0;
  bool   warned_not_found = false;
  bool   warned_bad_type  = false;
  // -----------------------------

  while (!g_should_exit.load()) {
    uint64_t expirations = 0;
    const auto bytes_read = read(timer_fd, &expirations, sizeof(expirations));
    if (bytes_read < 0) {
      if (errno == EINTR) {
        continue;
      }
      ERROR("Error reading Process timerfd (errno=%d).", errno);
      process_faulted->store(true);
      g_should_exit.store(true);
      break;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(expirations))) {
      ERROR("Unexpected Process timerfd read size %zd.", bytes_read);
      process_faulted->store(true);
      g_should_exit.store(true);
      break;
    }

    // ---- Jitter calculation ----
    const double current_time = NowMonotonicSeconds();
    double jitter = 0.0;
    if (have_last_time) {
      jitter = current_time - last_time - loop_period_sec;
    } else {
      // First sample: no previous timestamp to compare.
      jitter = 0.0;
      have_last_time = true;
    }
    last_time = current_time;

    if (!manager->Process()) {
      ERROR("fastcat manager faulted in timer thread. Exiting process loop.");
      process_faulted->store(true);
      g_should_exit.store(true);
      break;
    }

    double pos   = std::numeric_limits<double>::quiet_NaN();
    double vel   = std::numeric_limits<double>::quiet_NaN();
    double cur   = std::numeric_limits<double>::quiet_NaN();
    double power = std::numeric_limits<double>::quiet_NaN();
    std::vector<fastcat::DeviceState> states = manager->GetDeviceStates();
    for (const auto& s : states) {
      if (s.name != actuator_name) {
        continue;
      }
      pos   = s.gold_actuator_state.actual_position;
      vel   = s.gold_actuator_state.actual_velocity;
      cur   = s.gold_actuator_state.actual_current;
      power = s.gold_actuator_state.power;
      break;
    }

    telemetry_csv << current_time << "," << jitter << "," << expirations << ","
                  << pos << "," << vel << "," << cur << "," << power << "\n";
  }

  telemetry_csv.close();
  close(timer_fd);
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

  const auto loop_period = ComputeLoopPeriod(process_loop_frequency_hz);
  const auto start_time  = std::chrono::steady_clock::now();
  bool       sent_cmd    = false;
  std::atomic<bool> process_faulted(false);

  MSG("Starting Process timer thread at %f Hz.", process_loop_frequency_hz);
  std::thread process_thread(
      ProcessTimerThread, &fcat_manager, actuator_name, loop_period,
      &process_faulted);

  while (!g_should_exit.load()) {
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

        MSG("Queued ACTUATOR_PROF_POS_CMD for device '%s'"
            " at t=%f"
            " sec with target_position=%f"
            ", profile_velocity=%f"
            ", profile_accel=%f",
            actuator_name.c_str(), elapsed_sec, target_position,
            profile_velocity, profile_accel);
        sent_cmd = true;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (process_thread.joinable()) {
    process_thread.join();
  }

  fcat_manager.Shutdown();
  return process_faulted.load() ? 1 : 0;
}
