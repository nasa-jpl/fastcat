#ifndef FASTCAT_UTILS_PROFILE_UTILS_H_
#define FASTCAT_UTILS_PROFILE_UTILS_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "fastcat/fastcat.h"
#include "jsd/jsd_elmo_common_types.h"

// Logic shared by the elmo_vel_profile and elmo_pos_profile utilities. The
// argument parsing and profile math here are free of hardware dependencies so
// they can be exercised by utils/test_profile_utils.cc; the Run*() helpers
// drive a fastcat::Manager and require a live bus.
namespace profile_utils
{
// Elmo CiA-402 states that IsMotionFaultConditionMet() treats as nominal, i.e.
// states a subsequent run can issue a motion command from without tripping the
// off-nominal check.
constexpr jsd_elmo_state_machine_state_t kSafeElmoStates[] = {
    JSD_ELMO_STATE_MACHINE_STATE_SWITCH_ON_DISABLED,
    JSD_ELMO_STATE_MACHINE_STATE_READY_TO_SWITCH_ON,
    JSD_ELMO_STATE_MACHINE_STATE_SWITCHED_ON,
};

// Loop rate below which encoder quantization noise on actual_velocity is large
// enough to trip the drive's internal speed tracking limit. See
// doc/utilities.md, "Loop rate guidance".
constexpr double kMinRecommendedLoopRateHz = 64.0;

// Bus cycles pumped after ACTUATOR_RESET_CMD to let the reset take effect.
constexpr int kResetTicks = 30;

// Time allowed for the brake to disengage and the servo to enable.
constexpr double kBrakeDisengageTimeoutS = 2.0;

// Settle time between ExecuteAllDeviceResets() and reissuing the motion command.
constexpr double kRecoverSettleS = 0.5;

// Attempts to recover from the empirically observed SWITCHED_ON race before
// giving up.
constexpr int kMaxRecoverRetries = 5;

// Time allowed after ACTUATOR_HALT_CMD for the drive to walk itself down to one
// of kSafeElmoStates.
constexpr double kHaltSettleTimeoutS = 3.0;

// Tick intervals for the periodic console prints.
constexpr int kBrakeStatusPrintInterval = 50;
constexpr int kRunStatusPrintInterval   = 10;

// Sentinel meaning "no state logged yet", so the first observed state always
// prints. No real elmo_state_machine_state takes this value.
constexpr uint32_t kNoStateLogged = 0xFFFFFFFF;

// Set by the SIGINT/SIGTERM handler installed by InstallSignalHandlers().
extern std::atomic<bool> g_shutdown;

void InstallSignalHandlers();

// True if the drive has settled somewhere a subsequent run can start from.
bool IsSafeElmoState(uint32_t elmo_state_machine_state);

// Bus cycles spanning `seconds` at `loop_rate_hz`.
int TicksFromSeconds(double seconds, double loop_rate_hz);

// Every device `name` declared under buses[].devices[] in a fastcat config.
std::vector<std::string> CollectDeviceNames(const YAML::Node& node);

// True if `actuator_name` names a device in `node`. On failure the available
// names are reported on stderr.
bool ValidateActuatorName(const YAML::Node& node,
                          const std::string& actuator_name);

// "<YYYYMMDD>_<HHMMSS>_<tag>_telem.csv" in local time.
std::string MakeTelemetryFilename(const std::string& tag, std::time_t when);
std::string MakeTelemetryFilename(const std::string& tag);

// Duration of a trapezoidal position move, collapsing to a triangular profile
// when `distance` is too short to reach `max_velocity`.
double TrapezoidalMoveDuration(double distance, double accel,
                              double max_velocity);

// Duration of a velocity profile: ramp up, hold, ramp back down to rest.
double VelocityProfileDuration(double accel, double cruise_speed,
                               double cruise_duration);

// Parses `text` as a double, rejecting trailing garbage and empty input.
bool ParseDouble(const std::string& text, double& out);

struct VelProfileArgs {
  std::string config_path;
  std::string actuator_name;
  double      accel           = 0.0;
  double      cruise_speed    = 0.0;
  double      cruise_duration = 0.0;
};

struct PosProfileArgs {
  std::string config_path;
  std::string actuator_name;
  double      accel             = 0.0;
  double      max_velocity      = 0.0;
  double      relative_position = 0.0;
};

// Parse and range-check the command line. `args` excludes the program name. On
// failure `error` is set to a message describing the first problem found.
bool ParseVelProfileArgs(const std::vector<std::string>& args,
                         VelProfileArgs& out, std::string& error);
bool ParsePosProfileArgs(const std::vector<std::string>& args,
                         PosProfileArgs& out, std::string& error);

// The named actuator's state, or nullptr if it is not a gold actuator or not
// present in `states`.
const fastcat::GoldActuatorState* FindGoldActuatorState(
    const std::vector<fastcat::DeviceState>& states, const std::string& name);

// Queues ACTUATOR_RESET_CMD and pumps kResetTicks cycles to clear stale faults,
// logging Elmo state transitions. False if the bus faulted, in which case the
// caller should shut down rather than proceed to the profile.
bool RunResetPhase(fastcat::Manager& mgr, const std::string& actuator_name,
                   double loop_rate_hz);

// Queues ACTUATOR_HALT_CMD and pumps cycles until the Elmo state machine
// settles in one of kSafeElmoStates, so the next run starts cleanly. Warns on
// stderr if it does not settle within kHaltSettleTimeoutS.
void HaltAndSettle(fastcat::Manager& mgr, const std::string& actuator_name,
                   double loop_rate_hz);

// The per-tool parts of the control loop run by RunProfileLoop().
struct ProfileHooks {
  // Bus cycles to stay in the RUN phase before declaring the profile complete.
  int run_ticks = 0;

  // Motion command to queue on entering ISSUE, given the actuator's state at
  // that moment. Called again on each recovery retry.
  std::function<fastcat::DeviceCmd(const fastcat::GoldActuatorState&)> issue;

  // Appends one telemetry row.
  std::function<void(const fastcat::GoldActuatorState&, double unix_time_s,
                     double elapsed_s)>
      write_csv_row;

  // Periodic RUN phase console print, every kRunStatusPrintInterval ticks.
  std::function<void(const fastcat::GoldActuatorState&, double elapsed_s)>
      log_run_tick;

  // Optional extra console output once the RUN phase finishes.
  std::function<void(const fastcat::GoldActuatorState&)> on_run_complete;
};

struct ProfileResult {
  // actual_velocity is heavily quantized on low-resolution encoders (42
  // counts/rev x 64 Hz ~ 9.6 rad/s/count); the commanded value is the cleaner
  // indicator that the trapezoid actually ran.
  double peak_cmd_velocity    = 0.0;
  double peak_actual_velocity = 0.0;
};

// Runs the ISSUE -> BRAKE_DISENGAGE -> RUN phase machine at `loop_rate_hz`,
// recovering from faults up to kMaxRecoverRetries times, until the profile
// completes, the bus gives up, or g_shutdown is set.
ProfileResult RunProfileLoop(fastcat::Manager& mgr,
                             const std::string& actuator_name,
                             double loop_rate_hz, const ProfileHooks& hooks);

}  // namespace profile_utils

#endif
