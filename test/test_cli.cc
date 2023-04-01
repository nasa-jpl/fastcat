#include <pthread.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "fastcat/fastcat.h"
#include "jsd/jsd_timer.h"

pthread_mutex_t  fastcat_mutex = PTHREAD_MUTEX_INITIALIZER;
fastcat::Manager manager;
double           cli_start_time;
FILE*            file;

void print_header(std::vector<fastcat::DeviceState> states)
{
  fprintf(file, "rel_time_sec, ");
  for (auto state = states.begin(); state != states.end(); ++state) {
    switch (state->type) {
      case fastcat::EGD_STATE:

        fprintf(file, "%s_actual_position, ", state->name.c_str());
        fprintf(file, "%s_actual_velocity, ", state->name.c_str());
        fprintf(file, "%s_actual_current, ", state->name.c_str());

        fprintf(file, "%s_cmd_position, ", state->name.c_str());
        fprintf(file, "%s_cmd_velocity, ", state->name.c_str());
        fprintf(file, "%s_cmd_current, ", state->name.c_str());

        fprintf(file, "%s_target_reached, ", state->name.c_str());

        break;
      case fastcat::SIGNAL_GENERATOR_STATE:
        fprintf(file, "%s_output, ", state->name.c_str());
        break;
      case fastcat::COMMANDER_STATE:
        fprintf(file, "%s_enable, ", state->name.c_str());
        break;
      case fastcat::FUNCTION_STATE:
        fprintf(file, "%s_output, ", state->name.c_str());
        break;
      case fastcat::CONDITIONAL_STATE:
        fprintf(file, "%s_output, ", state->name.c_str());
        break;
      case fastcat::EL3602_STATE:
        fprintf(file, "%s_voltage_ch1, ", state->name.c_str());
        fprintf(file, "%s_voltage_ch2, ", state->name.c_str());
        fprintf(file, "%s_adc_value_ch1, ", state->name.c_str());
        fprintf(file, "%s_adc_value_ch2, ", state->name.c_str());
        break;
      case fastcat::SCHMITT_TRIGGER_STATE:
        fprintf(file, "%s_output, ", state->name.c_str());
        break;
      case fastcat::EL2124_STATE:
        fprintf(file, "%s_level_ch1, ", state->name.c_str());
        fprintf(file, "%s_level_ch2, ", state->name.c_str());
        fprintf(file, "%s_level_ch3, ", state->name.c_str());
        fprintf(file, "%s_level_ch4, ", state->name.c_str());
        break;
      case fastcat::FILTER_STATE:
        fprintf(file, "%s_output, ", state->name.c_str());
        break;
      case fastcat::FTS_STATE:
        fprintf(file, "%s_raw_fx, ", state->name.c_str());
        fprintf(file, "%s_raw_fy, ", state->name.c_str());
        fprintf(file, "%s_raw_fz, ", state->name.c_str());
        fprintf(file, "%s_raw_tx, ", state->name.c_str());
        fprintf(file, "%s_raw_ty, ", state->name.c_str());
        fprintf(file, "%s_raw_tz, ", state->name.c_str());
        fprintf(file, "%s_tared_fx, ", state->name.c_str());
        fprintf(file, "%s_tared_fy, ", state->name.c_str());
        fprintf(file, "%s_tared_fz, ", state->name.c_str());
        fprintf(file, "%s_tared_tx, ", state->name.c_str());
        fprintf(file, "%s_tared_ty, ", state->name.c_str());
        fprintf(file, "%s_tared_tz, ", state->name.c_str());
        break;
      case fastcat::JED0101_STATE:
        fprintf(file, "%s_status, ", state->name.c_str());
        fprintf(file, "%s_w_raw, ", state->name.c_str());
        fprintf(file, "%s_x_raw, ", state->name.c_str());
        fprintf(file, "%s_y_raw, ", state->name.c_str());
        fprintf(file, "%s_z_raw, ", state->name.c_str());
        fprintf(file, "%s_w, ", state->name.c_str());
        fprintf(file, "%s_x, ", state->name.c_str());
        fprintf(file, "%s_y, ", state->name.c_str());
        fprintf(file, "%s_z, ", state->name.c_str());
        fprintf(file, "%s_cmd, ", state->name.c_str());
        break;
      case fastcat::JED0200_STATE:
        fprintf(file, "%s_status, ", state->name.c_str());
        fprintf(file, "%s_ticks, ", state->name.c_str());
        fprintf(file, "%s_voltage_hv, ", state->name.c_str());
        fprintf(file, "%s_voltage_lv, ", state->name.c_str());
        fprintf(file, "%s_voltage_12v, ", state->name.c_str());
        fprintf(file, "%s_temp_ambient, ", state->name.c_str());
        fprintf(file, "%s_temp_actuator, ", state->name.c_str());
        fprintf(file, "%s_humidity, ", state->name.c_str());
        fprintf(file, "%s_pressure, ", state->name.c_str());
        fprintf(file, "%s_brake_current, ", state->name.c_str());
        fprintf(file, "%s_brake_cc_val, ", state->name.c_str());
        fprintf(file, "%s_cmd, ", state->name.c_str());
        break;
      case fastcat::EL3208_STATE:
        fprintf(file, "%s_output_ch1, ", state->name.c_str());
        fprintf(file, "%s_output_ch2, ", state->name.c_str());
        fprintf(file, "%s_output_ch3, ", state->name.c_str());
        fprintf(file, "%s_output_ch4, ", state->name.c_str());
        fprintf(file, "%s_output_ch5, ", state->name.c_str());
        fprintf(file, "%s_output_ch6, ", state->name.c_str());
        fprintf(file, "%s_output_ch7, ", state->name.c_str());
        fprintf(file, "%s_output_ch8, ", state->name.c_str());
        break;
      case fastcat::FAULTER_STATE:
        fprintf(file, "%s_enable, ", state->name.c_str());
        fprintf(file, "%s_fault_active, ", state->name.c_str());
        break;
      case fastcat::EGD_ACTUATOR_STATE:
      case fastcat::EPD_ACTUATOR_STATE:
        fprintf(file, "%s_actual_position, ", state->name.c_str());
        fprintf(file, "%s_actual_velocity, ", state->name.c_str());
        fprintf(file, "%s_actual_current, ", state->name.c_str());

        fprintf(file, "%s_cmd_position, ", state->name.c_str());
        fprintf(file, "%s_cmd_velocity, ", state->name.c_str());
        fprintf(file, "%s_cmd_current, ", state->name.c_str());

        fprintf(file, "%s_target_reached, ", state->name.c_str());
        fprintf(file, "%s_elmo_actual_position, ", state->name.c_str());
        fprintf(file, "%s_elmo_cmd_position, ", state->name.c_str());

        fprintf(file, "%s_motor_on, ", state->name.c_str());
        fprintf(file, "%s_servo_enabled, ", state->name.c_str());
        break;
      case fastcat::LINEAR_INTERPOLATION_STATE:
        fprintf(file, "%s_output, ", state->name.c_str());
        fprintf(file, "%s_is_saturated, ", state->name.c_str());

        break;
      default:
        break;
    }
  }
  fprintf(file, "\n");
}
void print_csv_data(std::vector<fastcat::DeviceState> states)
{
  double time = states.begin()->time - cli_start_time;

  fprintf(file, "%lf, ", time);
  for (auto state = states.begin(); state != states.end(); ++state) {
    switch (state->type) {
      case fastcat::EGD_STATE:

        fprintf(file, "%i, ", state->egd_state.actual_position);
        fprintf(file, "%i, ", state->egd_state.actual_velocity);
        fprintf(file, "%lf, ", state->egd_state.actual_current);

        fprintf(file, "%i, ", state->egd_state.cmd_position);
        fprintf(file, "%i, ", state->egd_state.cmd_velocity);
        fprintf(file, "%lf, ", state->egd_state.cmd_current);

        fprintf(file, "%u, ", state->egd_state.target_reached);
        break;
      case fastcat::SIGNAL_GENERATOR_STATE:
        fprintf(file, "%lf, ", state->signal_generator_state.output);
        break;
      case fastcat::COMMANDER_STATE:
        fprintf(file, "%u, ", state->commander_state.enable);
        break;
      case fastcat::FUNCTION_STATE:
        fprintf(file, "%lf, ", state->function_state.output);
        break;
      case fastcat::CONDITIONAL_STATE:
        fprintf(file, "%u, ", state->conditional_state.output);
        break;
      case fastcat::EL3602_STATE:
        fprintf(file, "%lf, ", state->el3602_state.voltage_ch1);
        fprintf(file, "%lf, ", state->el3602_state.voltage_ch2);
        fprintf(file, "%i, ", state->el3602_state.adc_value_ch1);
        fprintf(file, "%i, ", state->el3602_state.adc_value_ch2);
        break;
      case fastcat::SCHMITT_TRIGGER_STATE:
        fprintf(file, "%u, ", state->schmitt_trigger_state.output);
        break;
      case fastcat::EL2124_STATE:
        fprintf(file, "%u, ", state->el2124_state.level_ch1);
        fprintf(file, "%u, ", state->el2124_state.level_ch2);
        fprintf(file, "%u, ", state->el2124_state.level_ch3);
        fprintf(file, "%u, ", state->el2124_state.level_ch4);
        break;
      case fastcat::FILTER_STATE:
        fprintf(file, "%lf, ", state->filter_state.output);
        break;
      case fastcat::FTS_STATE:
        fprintf(file, "%lf, ", state->fts_state.raw_fx);
        fprintf(file, "%lf, ", state->fts_state.raw_fy);
        fprintf(file, "%lf, ", state->fts_state.raw_fz);
        fprintf(file, "%lf, ", state->fts_state.raw_tx);
        fprintf(file, "%lf, ", state->fts_state.raw_ty);
        fprintf(file, "%lf, ", state->fts_state.raw_tz);
        fprintf(file, "%lf, ", state->fts_state.tared_fx);
        fprintf(file, "%lf, ", state->fts_state.tared_fy);
        fprintf(file, "%lf, ", state->fts_state.tared_fz);
        fprintf(file, "%lf, ", state->fts_state.tared_tx);
        fprintf(file, "%lf, ", state->fts_state.tared_ty);
        fprintf(file, "%lf, ", state->fts_state.tared_tz);
        break;
      case fastcat::JED0101_STATE:
        fprintf(file, "%u, ", state->jed0101_state.status);
        fprintf(file, "%u, ", state->jed0101_state.w_raw);
        fprintf(file, "%u, ", state->jed0101_state.x_raw);
        fprintf(file, "%u, ", state->jed0101_state.y_raw);
        fprintf(file, "%u, ", state->jed0101_state.z_raw);
        fprintf(file, "%lf, ", state->jed0101_state.w);
        fprintf(file, "%lf, ", state->jed0101_state.x);
        fprintf(file, "%lf, ", state->jed0101_state.y);
        fprintf(file, "%lf, ", state->jed0101_state.z);
        fprintf(file, "%u, ", state->jed0101_state.cmd);
        break;
      case fastcat::JED0200_STATE:
        fprintf(file, "%u, ", state->jed0200_state.status);
        fprintf(file, "%u, ", state->jed0200_state.ticks);
        fprintf(file, "%f, ", state->jed0200_state.voltage_hv);
        fprintf(file, "%f, ", state->jed0200_state.voltage_lv);
        fprintf(file, "%f, ", state->jed0200_state.voltage_12v);
        fprintf(file, "%f, ", state->jed0200_state.temp_ambient);
        fprintf(file, "%f, ", state->jed0200_state.temp_actuator);
        fprintf(file, "%f, ", state->jed0200_state.humidity);
        fprintf(file, "%f, ", state->jed0200_state.pressure);
        fprintf(file, "%u, ", state->jed0200_state.brake_current);
        fprintf(file, "%u, ", state->jed0200_state.brake_cc_val);
        fprintf(file, "%u, ", state->jed0200_state.cmd);
        break;
      case fastcat::EL3208_STATE:
        fprintf(file, "%lf, ", state->el3208_state.output_ch1);
        fprintf(file, "%lf, ", state->el3208_state.output_ch2);
        fprintf(file, "%lf, ", state->el3208_state.output_ch3);
        fprintf(file, "%lf, ", state->el3208_state.output_ch4);
        fprintf(file, "%lf, ", state->el3208_state.output_ch5);
        fprintf(file, "%lf, ", state->el3208_state.output_ch6);
        fprintf(file, "%lf, ", state->el3208_state.output_ch7);
        fprintf(file, "%lf, ", state->el3208_state.output_ch8);
        break;
      case fastcat::FAULTER_STATE:
        fprintf(file, "%u, ", state->faulter_state.enable);
        fprintf(file, "%u, ", state->faulter_state.fault_active);
        break;
      case fastcat::EGD_ACTUATOR_STATE:
        fprintf(file, "%lf, ", state->egd_actuator_state.actual_position);
        fprintf(file, "%lf, ", state->egd_actuator_state.actual_velocity);
        fprintf(file, "%lf, ", state->egd_actuator_state.actual_current);

        fprintf(file, "%lf, ", state->egd_actuator_state.cmd_position);
        fprintf(file, "%lf, ", state->egd_actuator_state.cmd_velocity);
        fprintf(file, "%lf, ", state->egd_actuator_state.cmd_current);

        fprintf(file, "%u, ", state->egd_actuator_state.target_reached);

        fprintf(file, "%i, ", state->egd_actuator_state.elmo_actual_position);
        fprintf(file, "%i, ", state->egd_actuator_state.elmo_cmd_position);

        fprintf(file, "%u, ", state->egd_actuator_state.motor_on);
        fprintf(file, "%u, ", state->egd_actuator_state.servo_enabled);
        break;
      case fastcat::EPD_ACTUATOR_STATE:
        fprintf(file, "%lf, ", state->epd_actuator_state.actual_position);
        fprintf(file, "%lf, ", state->epd_actuator_state.actual_velocity);
        fprintf(file, "%lf, ", state->epd_actuator_state.actual_current);

        fprintf(file, "%lf, ", state->epd_actuator_state.cmd_position);
        fprintf(file, "%lf, ", state->epd_actuator_state.cmd_velocity);
        fprintf(file, "%lf, ", state->epd_actuator_state.cmd_current);

        fprintf(file, "%u, ", state->epd_actuator_state.target_reached);

        fprintf(file, "%i, ", state->epd_actuator_state.elmo_actual_position);
        fprintf(file, "%i, ", state->epd_actuator_state.elmo_cmd_position);

        fprintf(file, "%u, ", state->epd_actuator_state.motor_on);
        fprintf(file, "%u, ", state->epd_actuator_state.servo_enabled);
        break;
      case fastcat::LINEAR_INTERPOLATION_STATE:
        fprintf(file, "%lf, ", state->linear_interpolation_state.output);
        fprintf(file, "%u, ", state->linear_interpolation_state.is_saturated);
        break;

      default:
        break;
    }
  }
  fprintf(file, "\n");
}

bool quit = false;  // quit flag
void on_signal(int /* signal */)
{
  MSG("Received ctrl-c. Closing down.");
  quit = true;
}

void* cli_process(void*)
{
  fastcat::DeviceCmd cmd;
  while (!quit) {
    char* line = readline("> ");
    if (!line) break;
    if (*line) add_history(line);
    MSG("user input is: %s", line);

    // parse commands
    std::vector<std::string> tokens;
    std::stringstream        ss(line);
    std::string              word;
    while (getline(ss, word, ' ')) {
      tokens.push_back(word);
    }

    if (tokens.size() < 1) {
      continue;
    }

    // make sure all fields are zero'ed
    cmd = fastcat::DeviceCmd{};

    if (tokens[0].compare("exit") == 0 || tokens[0].compare("quit") == 0) {
      quit = true;
      free(line);
      return NULL;
    } else if (tokens[0].compare("egd_prof_pos") == 0 && tokens.size() == 8) {
      std::string name     = tokens[1];
      int32_t     pos      = atoi(tokens[2].c_str());
      uint32_t    vel      = atoi(tokens[3].c_str());
      uint32_t    end_vel  = atoi(tokens[4].c_str());
      uint32_t    accel    = atoi(tokens[5].c_str());
      uint32_t    decel    = atoi(tokens[6].c_str());
      uint8_t     relative = atoi(tokens[7].c_str());

      MSG("Issuing egd_prof_pos %s %i %u %u %u %u %u", name.c_str(), pos, vel,
          end_vel, accel, decel, relative);

      cmd.name                              = name;
      cmd.type                              = fastcat::EGD_PROF_POS_CMD;
      cmd.egd_prof_pos_cmd.target_position  = pos;
      cmd.egd_prof_pos_cmd.profile_velocity = vel;
      cmd.egd_prof_pos_cmd.end_velocity     = end_vel;
      cmd.egd_prof_pos_cmd.profile_accel    = accel;
      cmd.egd_prof_pos_cmd.profile_decel    = decel;
      cmd.egd_prof_pos_cmd.relative         = relative;

    } else if (tokens[0].compare("egd_prof_vel") == 0 && tokens.size() == 5) {
      std::string name  = tokens[1];
      int32_t     vel   = atoi(tokens[2].c_str());
      uint32_t    accel = atoi(tokens[3].c_str());
      uint32_t    decel = atoi(tokens[4].c_str());

      MSG("Issuing egd_prof_vel %s %i %u %u", name.c_str(), vel, accel, decel);

      cmd.name                             = name;
      cmd.type                             = fastcat::EGD_PROF_VEL_CMD;
      cmd.egd_prof_vel_cmd.target_velocity = vel;
      cmd.egd_prof_vel_cmd.profile_accel   = accel;
      cmd.egd_prof_vel_cmd.profile_decel   = decel;

    } else if (tokens[0].compare("egd_prof_torque") == 0 &&
               tokens.size() == 3) {
      std::string name   = tokens[1];
      double      torque = atof(tokens[2].c_str());

      MSG("Issuing egd_prof_torque %s %lf", name.c_str(), torque);

      cmd.name                                   = name;
      cmd.type                                   = fastcat::EGD_PROF_TORQUE_CMD;
      cmd.egd_prof_torque_cmd.target_torque_amps = torque;

    } else if (tokens[0].compare("egd_reset") == 0 && tokens.size() == 2) {
      MSG("Issuing egd_reset");
      cmd.name = tokens[1];
      cmd.type = fastcat::EGD_RESET_CMD;
    } else if (tokens[0].compare("commander_enable") == 0 &&
               tokens.size() == 3) {
      MSG("Issuing commander_enable");
      cmd.name                          = tokens[1];
      cmd.commander_enable_cmd.duration = atof(tokens[2].c_str());
      cmd.type                          = fastcat::COMMANDER_ENABLE_CMD;

    } else if (tokens[0].compare("el2124_write_channel") == 0 &&
               tokens.size() == 4) {
      MSG("Issuing el2124_write_channel command");
      cmd.name                             = tokens[1];
      cmd.el2124_write_channel_cmd.channel = atoi(tokens[2].c_str());
      cmd.el2124_write_channel_cmd.level   = atoi(tokens[3].c_str());
      cmd.type                             = fastcat::EL2124_WRITE_CHANNEL_CMD;

    } else if (tokens[0].compare("jed0101_set_cmd_value") == 0 &&
               tokens.size() == 3) {
      MSG("Issuing jed0101_set_cmd_value command");
      cmd.name                          = tokens[1];
      cmd.jed0101_set_cmd_value_cmd.cmd = atoi(tokens[2].c_str());
      cmd.type                          = fastcat::JED0101_SET_CMD_VALUE_CMD;

    } else if (tokens[0].compare("jed0200_set_cmd_value") == 0 &&
               tokens.size() == 3) {
      MSG("Issuing jed0200_set_cmd_value command");
      cmd.name                          = tokens[1];
      cmd.jed0200_set_cmd_value_cmd.cmd = atoi(tokens[2].c_str());
      cmd.type                          = fastcat::JED0200_SET_CMD_VALUE_CMD;

    } else if (tokens[0].compare("actuator_set_output_position") == 0 &&
               tokens.size() == 3) {
      MSG("Issuing actuator_set_output_position command");
      cmd.name                                      = tokens[1];
      cmd.actuator_set_output_position_cmd.position = atof(tokens[2].c_str());
      cmd.type = fastcat::ACTUATOR_SET_OUTPUT_POSITION_CMD;

    } else if (tokens[0].compare("actuator_prof_pos") == 0 &&
               tokens.size() == 6) {
      MSG("Issuing actuator_prof_pos command");
      cmd.name                                   = tokens[1];
      cmd.actuator_prof_pos_cmd.target_position  = atof(tokens[2].c_str());
      cmd.actuator_prof_pos_cmd.profile_velocity = atof(tokens[3].c_str());
      cmd.actuator_prof_pos_cmd.profile_accel    = atof(tokens[4].c_str());
      cmd.actuator_prof_pos_cmd.relative         = atoi(tokens[5].c_str());
      cmd.type = fastcat::ACTUATOR_PROF_POS_CMD;

    } else if (tokens[0].compare("reset") == 0) {
      manager.ExecuteAllDeviceResets();
      free(line);
      continue;

    } else if (tokens[0].compare("bus_recovery") == 0 && tokens.size() == 2) {
      MSG("Calling bus_recovery for %s", tokens[1].c_str());
      pthread_mutex_lock(&fastcat_mutex);
      manager.RecoverBus(tokens[1]);
      pthread_mutex_unlock(&fastcat_mutex);
      free(line);
      continue;
    } else {
      continue;
    }

    // if valid, lock mutex and issue command
    pthread_mutex_lock(&fastcat_mutex);
    manager.QueueCommand(cmd);
    pthread_mutex_unlock(&fastcat_mutex);

    free(line);
  }
  return NULL;
}

int main(int argc, char* argv[])
{
  if (argc != 2) {
    ERROR("Pass input yaml file as first argument");
    return 0;
  }

  umask(0000);
  file = fopen("/tmp/fastcat_cli.csv", "w");
  if (!file) {
    ERROR("Could not open /tmp/fastcat_cli.csv for writing");
    return 0;
  }

  signal(SIGINT, on_signal);

  std::string filepath(argv[1]);
  MSG("loading Yaml from %s", filepath.c_str());
  YAML::Node node = YAML::LoadFile(filepath);
  if (!manager.ConfigFromYaml(node)) {
    ERROR("Could not configure Fastcat Manager");
    return 0;
  }

  pthread_t cli_thread;
  int       retval;
  double    loop_rate = manager.GetTargetLoopRate();

  jsd_timer_t* timer = jsd_timer_alloc();
  jsd_timer_init_ex(timer, 1e9 / loop_rate, JSD_TIMER_ANY_CPU, false, false);
  cli_start_time = jsd_time_get_time_sec();

  std::vector<fastcat::DeviceState> states;

  manager.Process();
  states = manager.GetDeviceStates();

  retval = pthread_create(&cli_thread, NULL, cli_process, NULL);
  if (retval) {
    ERROR("Thread creation failed: %d\n", retval);
    return 1;
  }

  print_header(states);

  while (!quit) {
    jsd_timer_process(timer);

    pthread_mutex_lock(&fastcat_mutex);
    manager.Process();
    pthread_mutex_unlock(&fastcat_mutex);

    states = manager.GetDeviceStates();
    print_csv_data(states);
  }
  manager.Shutdown();
  return 1;
}
