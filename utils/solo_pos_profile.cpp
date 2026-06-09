#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>
#include "fastcat/fastcat.h"

std::atomic<bool> g_shutdown{false};

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
    g_shutdown = true;
}

bool validate_actuator_name(const YAML::Node& node, const std::string& actuator_name) {
    std::vector<std::string> available_names;

    if (node["buses"]) {
        for (const auto& bus : node["buses"]) {
            if (bus["devices"]) {
                for (const auto& device : bus["devices"]) {
                    if (device["name"]) {
                        std::string name = device["name"].as<std::string>();
                        available_names.push_back(name);
                        if (name == actuator_name) {
                            return true;
                        }
                    }
                }
            }
        }
    }

    std::cerr << "Error: Actuator '" << actuator_name << "' not found in YAML config." << std::endl;
    std::cerr << "Available actuator names: ";
    for (size_t i = 0; i < available_names.size(); ++i) {
        std::cerr << "'" << available_names[i] << "'";
        if (i < available_names.size() - 1) std::cerr << ", ";
    }
    std::cerr << std::endl;
    return false;
}

std::string get_timestamp_filename() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&time_t_now, &tm_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_pos_prof_telem.csv";
    return oss.str();
}

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "Usage: " << argv[0] << " <config_path> <actuator_name> <accel> <max_velocity> <relative_position>" << std::endl;
        std::cerr << "  config_path: relative path to fastcat YAML config" << std::endl;
        std::cerr << "  actuator_name: name of actuator in YAML" << std::endl;
        std::cerr << "  accel: acceleration/deceleration (rad/s²)" << std::endl;
        std::cerr << "  max_velocity: maximum velocity (rad/s)" << std::endl;
        std::cerr << "  relative_position: position change (rad, can be negative)" << std::endl;
        return 1;
    }

    std::string config_path = argv[1];
    std::string actuator_name = argv[2];
    double accel = std::atof(argv[3]);
    double max_velocity = std::atof(argv[4]);
    double relative_position = std::atof(argv[5]);

    if (accel <= 0) {
        std::cerr << "Error: acceleration must be positive" << std::endl;
        return 1;
    }
    if (max_velocity <= 0) {
        std::cerr << "Error: max_velocity must be positive" << std::endl;
        return 1;
    }
    if (relative_position == 0) {
        std::cerr << "Error: relative_position cannot be zero" << std::endl;
        return 1;
    }

    // Load YAML config
    YAML::Node node;
    try {
        node = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file '" << config_path << "': " << e.what() << std::endl;
        return 1;
    }

    // Validate actuator name
    if (!validate_actuator_name(node, actuator_name)) {
        return 1;
    }

    // Install signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Initialize fastcat manager
    fastcat::Manager mgr;
    std::cout << "Initializing fastcat manager..." << std::endl;
    if (!mgr.ConfigFromYaml(node)) {
        std::cerr << "Error: Failed to configure manager from YAML" << std::endl;
        return 1;
    }

    double loop_rate = mgr.GetTargetLoopRate();
    std::cout << "Loop rate: " << loop_rate << " Hz" << std::endl;
    if (loop_rate < 64.0) {
        std::cerr << "Warning: Loop rate " << loop_rate << " Hz is below minimum recommended 64 Hz" << std::endl;
    }

    auto period = std::chrono::duration<double>(1.0 / loop_rate);

    // Reset the actuator to clear any previous faults
    std::cout << "Resetting actuator '" << actuator_name << "'..." << std::endl;

    // Send reset command via queue
    fastcat::DeviceCmd reset_cmd;
    reset_cmd.name = actuator_name;
    reset_cmd.type = fastcat::ACTUATOR_RESET_CMD;
    mgr.QueueCommand(reset_cmd);

    // Process several cycles at the configured loop rate to let the reset command execute
    auto reset_start = std::chrono::steady_clock::now();
    bool reset_failed = false;
    uint32_t last_elmo_state = 0xFFFFFFFF;
    for (int i = 0; i < 30; ++i) {
        if (!mgr.Process()) {
            std::cerr << "Error: Manager process failed during reset phase at iteration " << i << std::endl;
            std::cerr << "This likely indicates a hardware/communication issue with the drive." << std::endl;
            std::cerr << "Check: 1) Motor is connected  2) Drive has power  3) EtherCAT cable is secure" << std::endl;
            reset_failed = true;
            break;
        }
        // Log Elmo state machine state changes during warmup
        auto warmup_states = mgr.GetDeviceStates();
        for (const auto& s : warmup_states) {
            if (s.name == actuator_name && s.type == fastcat::GOLD_ACTUATOR_STATE) {
                if (s.gold_actuator_state.elmo_state_machine_state != last_elmo_state) {
                    std::cout << "  [warmup tick " << i << "] elmo_sms=0x" << std::hex
                              << s.gold_actuator_state.elmo_state_machine_state
                              << " act_sms=0x" << s.gold_actuator_state.actuator_state_machine_state
                              << std::dec
                              << " servo=" << (int)s.gold_actuator_state.servo_enabled
                              << " motor_on=" << (int)s.gold_actuator_state.motor_on
                              << " jsd_fault=" << s.gold_actuator_state.jsd_fault_code << std::endl;
                    last_elmo_state = s.gold_actuator_state.elmo_state_machine_state;
                }
            }
        }
        std::this_thread::sleep_for(period);
    }
    auto reset_duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - reset_start).count();

    if (reset_failed) {
        mgr.Shutdown();
        return 1;
    }

    std::cout << "Reset phase completed in " << std::fixed << std::setprecision(1)
              << reset_duration << "s, proceeding to warmup..." << std::endl;

    auto next_tick = std::chrono::steady_clock::now() + period;

    // Open CSV file for telemetry
    std::string csv_filename = get_timestamp_filename();
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        std::cerr << "Error: Failed to open CSV file: " << csv_filename << std::endl;
        mgr.Shutdown();
        return 1;
    }

    // Write CSV header
    csv_file << "unix_time_s,relative_time_s,cmd_position_rad,actual_position_rad,actual_velocity_rad_s,actual_current_A" << std::endl;
    std::cout << "Writing telemetry to: " << csv_filename << std::endl;

    // State machine variables
    enum Phase { ISSUE, BRAKE_DISENGAGE, RUN, RECOVER, DONE };
    Phase phase = ISSUE;
    int tick_count = 0;
    const int brake_disengage_ticks = static_cast<int>(2.0 * loop_rate);  // 2s wait for brake/enable

    // Calculate expected motion duration for a trapezoidal position profile
    double distance = std::abs(relative_position);
    double accel_time = max_velocity / accel;
    double accel_distance = 0.5 * accel * accel_time * accel_time;
    double cruise_time = 0.0;
    double expected_duration = 0.0;

    if (distance < 2.0 * accel_distance) {
        // Triangular profile - never reach max_velocity
        accel_time = std::sqrt(distance / accel);
        expected_duration = 2.0 * accel_time;
    } else {
        // Trapezoidal profile
        double cruise_distance = distance - 2.0 * accel_distance;
        cruise_time = cruise_distance / max_velocity;
        expected_duration = 2.0 * accel_time + cruise_time;
    }

    int run_ticks = static_cast<int>((expected_duration + 1.0) * loop_rate);  // +1s margin
    int run_tick_count = 0;

    // Retry/recovery counters for empirical SWITCHED_ON race recovery
    const int MAX_RETRIES = 5;
    const int recover_settle_ticks = static_cast<int>(0.5 * loop_rate);
    int retry_count = 0;
    int recover_tick_count = 0;

    double peak_actual_velocity = 0.0;
    double peak_cmd_velocity = 0.0;
    double initial_position = 0.0;
    double target_position = 0.0;
    bool motor_ready = false;
    auto start_time = std::chrono::steady_clock::now();

    std::cout << "\nStarting control loop at " << loop_rate << " Hz..." << std::endl;
    std::cout << "Profile parameters: accel=" << accel << " rad/s², max_vel=" << max_velocity
              << " rad/s, distance=" << distance << " rad" << std::endl;
    std::cout << "Expected motion duration: " << expected_duration << " s" << std::endl;

    // Main control loop
    while (phase != DONE && !g_shutdown) {
        // Process EtherCAT cycle. If fault occurred and we have retries, drop into RECOVER.
        bool process_ok = mgr.Process();
        if (!process_ok) {
            if (phase == RUN) {
                std::cerr << "Error: Bus faulted during RUN phase" << std::endl;
                phase = DONE;
            } else if (phase != RECOVER && retry_count < MAX_RETRIES) {
                std::cerr << "Bus faulted during " << (phase == ISSUE ? "ISSUE" : "BRAKE_DISENGAGE")
                          << " (attempt " << (retry_count + 1) << "/" << MAX_RETRIES
                          << "), entering RECOVER..." << std::endl;
                phase = RECOVER;
                recover_tick_count = 0;
            } else {
                std::cerr << "Error: Manager process failed and retries exhausted" << std::endl;
                break;
            }
        }

        // Get device states
        auto states = mgr.GetDeviceStates();
        fastcat::GoldActuatorState* act_state = nullptr;

        for (auto& state : states) {
            if (state.name == actuator_name && state.type == fastcat::GOLD_ACTUATOR_STATE) {
                act_state = &state.gold_actuator_state;
                break;
            }
        }

        if (!act_state) {
            std::cerr << "Error: Could not find actuator state for '" << actuator_name << "'" << std::endl;
            break;
        }

        // Get current system time relative to start
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();

        // Get unix timestamp
        auto unix_time = std::chrono::system_clock::now();
        double unix_time_s = std::chrono::duration<double>(unix_time.time_since_epoch()).count();

        // Write telemetry to CSV
        csv_file << std::fixed << std::setprecision(6)
                 << unix_time_s << ","
                 << elapsed_time << ","
                 << act_state->cmd_position << ","
                 << act_state->actual_position << ","
                 << act_state->actual_velocity << ","
                 << act_state->actual_current << std::endl;

        // Track peak velocities. actual_velocity is heavily quantized on low-
        // resolution encoders (42 counts/rev × 64 Hz ≈ 9.6 rad/s/count); the
        // commanded value is the cleaner indicator that the trap ran.
        if (std::abs(act_state->actual_velocity) > peak_actual_velocity) {
            peak_actual_velocity = std::abs(act_state->actual_velocity);
        }
        if (std::abs(act_state->cmd_velocity) > peak_cmd_velocity) {
            peak_cmd_velocity = std::abs(act_state->cmd_velocity);
        }

        // State machine logic
        switch (phase) {
            case ISSUE: {
                // Capture initial position and calculate target
                initial_position = act_state->actual_position;
                target_position = initial_position + relative_position;

                // Issue profile position command immediately
                // This triggers brake disengagement and motor enable
                fastcat::DeviceCmd cmd;
                cmd.name = actuator_name;
                cmd.type = fastcat::ACTUATOR_PROF_POS_CMD;
                cmd.actuator_prof_pos_cmd.target_position = target_position;
                cmd.actuator_prof_pos_cmd.profile_velocity = max_velocity;
                cmd.actuator_prof_pos_cmd.profile_accel = accel;
                cmd.actuator_prof_pos_cmd.relative = 0;  // Absolute position (we calculated target)
                mgr.QueueCommand(cmd);

                std::cout << "Profile position command issued" << std::endl;
                std::cout << "  Initial position: " << initial_position << " rad" << std::endl;
                std::cout << "  Target position: " << target_position << " rad" << std::endl;
                std::cout << "  Relative move: " << relative_position << " rad" << std::endl;
                std::cout << "Waiting for brake disengagement and motor enable..." << std::endl;
                phase = BRAKE_DISENGAGE;
                tick_count = 0;
                break;
            }

            case BRAKE_DISENGAGE:
                // Wait for motor to enable (brake disengage + servo enable)
                if (!motor_ready && act_state->servo_enabled && act_state->motor_on) {
                    std::cout << "Motor enabled (servo_enabled=1, motor_on=1)" << std::endl;
                    motor_ready = true;
                    std::cout << "Executing motion profile..." << std::endl;
                    phase = RUN;
                    run_tick_count = 0;
                    break;
                }

                // Print status periodically
                if (tick_count % 50 == 0 && tick_count > 0) {
                    std::cout << "  Waiting for motor enable: servo=" << (int)act_state->servo_enabled
                              << " motor_on=" << (int)act_state->motor_on
                              << " state=0x" << std::hex << act_state->actuator_state_machine_state << std::dec
                              << std::endl;
                }

                // Check for faults — let the top-of-loop process_ok branch handle recovery
                if (mgr.IsFaulted()) {
                    std::cerr << "Manager faulted during brake disengagement" << std::endl;
                    std::cerr << "  Fault code: " << act_state->fastcat_fault_code
                              << "  EMCY: 0x" << std::hex << act_state->emcy_error_code << std::dec
                              << "  elmo_sms=0x" << std::hex << act_state->elmo_state_machine_state << std::dec
                              << std::endl;
                    if (retry_count < MAX_RETRIES) {
                        phase = RECOVER;
                        recover_tick_count = 0;
                    } else {
                        std::cerr << "Retries exhausted." << std::endl;
                        phase = DONE;
                    }
                    break;
                }

                tick_count++;
                if (tick_count >= brake_disengage_ticks) {
                    std::cerr << "Error: Motor did not enable within " << (brake_disengage_ticks / loop_rate) << "s" << std::endl;
                    std::cerr << "servo_enabled=" << (int)act_state->servo_enabled
                              << " motor_on=" << (int)act_state->motor_on
                              << " state=0x" << std::hex << act_state->actuator_state_machine_state << std::dec
                              << std::endl;
                    phase = DONE;
                }
                break;

            case RUN:
                // Trap is self-driving once issued — do not re-queue PROF_POS or
                // we restart the profile every tick.

                // Print position and velocity periodically
                if (run_tick_count % 10 == 0) {
                    std::cout << "  t=" << std::fixed << std::setprecision(2) << elapsed_time
                              << "s  pos=" << std::setprecision(3) << act_state->actual_position
                              << " rad  vel=" << act_state->actual_velocity
                              << " rad/s  current=" << act_state->actual_current << " A" << std::endl;
                }

                run_tick_count++;
                if (run_tick_count >= run_ticks) {
                    std::cout << "Motion profile complete" << std::endl;
                    std::cout << "  Final position: " << act_state->actual_position << " rad" << std::endl;
                    std::cout << "  Position error: " << (target_position - act_state->actual_position) << " rad" << std::endl;
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
                              << (retry_count + 1) << "/" << MAX_RETRIES << ")" << std::endl;
                    mgr.ExecuteAllDeviceResets();
                }
                recover_tick_count++;
                if (recover_tick_count >= recover_settle_ticks) {
                    retry_count++;
                    std::cout << "RECOVER: settle complete, reissuing PROF_VEL"
                              << "  elmo_sms=0x" << std::hex << act_state->elmo_state_machine_state << std::dec
                              << "  servo=" << (int)act_state->servo_enabled
                              << "  motor_on=" << (int)act_state->motor_on
                              << std::endl;
                    motor_ready = false;
                    tick_count = 0;
                    phase = ISSUE;
                }
                break;

            case DONE:
                break;
        }

        // Sleep until next tick
        std::this_thread::sleep_until(next_tick);
        next_tick += period;
    }

    csv_file.close();

    // Graceful halt: queue HALT_CMD and pump cycles until the Elmo CIA-402
    // state machine settles at a non-fault, non-QUICK_STOP state so the next
    // run can issue PROF_VEL without tripping the off-nominal check.
    // Safe target states (v0.13.15 IsMotionFaultConditionMet does not fault on these):
    //   0x40 SWITCH_ON_DISABLED, 0x21 READY_TO_SWITCH_ON, 0x23 SWITCHED_ON
    // Off-nominal that we want to leave behind:
    //   0x07 QUICK_STOP_ACTIVE, 0x08 FAULT, 0x0F FAULT_REACTION_ACTIVE
    std::cout << "\nGraceful halt: waiting for drive to reach safe state..." << std::endl;
    {
        fastcat::DeviceCmd halt_cmd;
        halt_cmd.name = actuator_name;
        halt_cmd.type = fastcat::ACTUATOR_HALT_CMD;
        mgr.QueueCommand(halt_cmd);
    }

    const int max_settle_ticks = static_cast<int>(3.0 * loop_rate);
    uint32_t last_logged_sms = 0xFFFFFFFF;
    bool reached_safe_state = false;
    uint32_t final_sms = 0;
    for (int i = 0; i < max_settle_ticks; ++i) {
        // Ignore Process() return — drive may transit through fault states
        // (e.g. QUICK_STOP_ACTIVE) on the way down, and jsd_egd will auto-issue
        // FAULT_RESET to walk it back to SWITCH_ON_DISABLED.
        mgr.Process();

        auto states = mgr.GetDeviceStates();
        for (const auto& s : states) {
            if (s.name == actuator_name && s.type == fastcat::GOLD_ACTUATOR_STATE) {
                final_sms = s.gold_actuator_state.elmo_state_machine_state;
                if (final_sms != last_logged_sms) {
                    std::cout << "  shutdown tick " << i << ": elmo_sms=0x"
                              << std::hex << final_sms << std::dec << std::endl;
                    last_logged_sms = final_sms;
                }
                if (final_sms == 0x40 || final_sms == 0x21 || final_sms == 0x23) {
                    reached_safe_state = true;
                }
                break;
            }
        }
        if (reached_safe_state) break;
        std::this_thread::sleep_for(period);
    }

    if (reached_safe_state) {
        std::cout << "Drive reached safe state (elmo_sms=0x" << std::hex << final_sms
                  << std::dec << "); next run can start cleanly." << std::endl;
    } else {
        std::cerr << "Warning: drive did not reach a safe state within "
                  << (max_settle_ticks / loop_rate) << "s (last elmo_sms=0x"
                  << std::hex << final_sms << std::dec << ")." << std::endl;
        std::cerr << "  If next run reports 'off nominal', power-cycle the drive first."
                  << std::endl;
    }

    std::cout << "\nShutting down..." << std::endl;
    mgr.Shutdown();

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Commanded move:       " << std::fixed << std::setprecision(3) << relative_position << " rad" << std::endl;
    std::cout << "Peak cmd velocity:    " << peak_cmd_velocity << " rad/s" << std::endl;
    std::cout << "Peak actual velocity: " << peak_actual_velocity
              << " rad/s (noisy at low encoder resolution)" << std::endl;
    std::cout << "Telemetry saved to: " << csv_filename << std::endl;

    return 0;
}
