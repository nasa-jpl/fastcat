// Include related header (for cc files)
#include "fastcat/jsd/actuator_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>

// Include external then project includes
#include "jsd/jsd.h"

fastcat::ActuatorOffline::ActuatorOffline()
{
  MSG_DEBUG("Constructed ActuatorOffline");
}

void fastcat::ActuatorOffline::EgdRead()
{
  // no-op
}

void fastcat::ActuatorOffline::EgdSetConfig()
{
  // no-op
}
void fastcat::ActuatorOffline::EgdProcess()
{
  // no-op
}

void fastcat::ActuatorOffline::EgdReset()
{
  // no-op
}

void fastcat::ActuatorOffline::EgdHalt()
{
  // no-op
}

void fastcat::ActuatorOffline::EgdSetPeakCurrent(double /* current */)
{
  // no-op
}

void fastcat::ActuatorOffline::EgdCSP(jsd_egd_motion_command_csp_t jsd_csp_cmd)
{
  jsd_egd_state_.cmd_position = jsd_csp_cmd.target_position;
  jsd_egd_state_.cmd_velocity = 0;
  jsd_egd_state_.cmd_current  = 0;

  jsd_egd_state_.cmd_ff_position = jsd_csp_cmd.position_offset;
  jsd_egd_state_.cmd_ff_velocity = jsd_csp_cmd.velocity_offset;
  jsd_egd_state_.cmd_ff_current  = jsd_csp_cmd.torque_offset_amps;

  // simulate actuals
  jsd_egd_state_.actual_position =
      jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position;
  jsd_egd_state_.actual_velocity =
      jsd_egd_state_.cmd_velocity + jsd_egd_state_.cmd_ff_velocity;
  jsd_egd_state_.actual_current =
      jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current;
}

void fastcat::ActuatorOffline::EgdCSV(jsd_egd_motion_command_csv_t jsd_csv_cmd)
{
  jsd_egd_state_.cmd_position = 0;
  jsd_egd_state_.cmd_velocity = jsd_csv_cmd.target_velocity;
  jsd_egd_state_.cmd_current  = 0;

  jsd_egd_state_.cmd_ff_position = 0;
  jsd_egd_state_.cmd_ff_velocity = jsd_csv_cmd.velocity_offset;
  jsd_egd_state_.cmd_ff_current  = jsd_csv_cmd.torque_offset_amps;

  // simulate actuals
  jsd_egd_state_.actual_velocity =
      jsd_egd_state_.cmd_velocity + jsd_egd_state_.cmd_ff_velocity;
  jsd_egd_state_.actual_current =
      jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current;
  jsd_egd_state_.actual_position +=
      jsd_egd_state_.actual_velocity * loop_period_;  // integrated
}

void fastcat::ActuatorOffline::EgdCST(jsd_egd_motion_command_cst_t jsd_cst_cmd)
{
  jsd_egd_state_.cmd_position = 0;
  jsd_egd_state_.cmd_velocity = 0;
  jsd_egd_state_.cmd_current  = jsd_cst_cmd.target_torque_amps;

  jsd_egd_state_.cmd_ff_position = 0;
  jsd_egd_state_.cmd_ff_velocity = 0;
  jsd_egd_state_.cmd_ff_current  = jsd_cst_cmd.torque_offset_amps;

  // simulate actuals
  jsd_egd_state_.actual_position =
      jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position;
  jsd_egd_state_.actual_current =
      jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current;

  double pct = jsd_egd_state_.actual_current / continuous_current_limit_amps_;
  jsd_egd_state_.actual_velocity =
      pct * max_speed_eu_per_sec_;  // sure, why not
  jsd_egd_state_.actual_position +=
      jsd_egd_state_.actual_velocity * loop_period_;  // integrated
}
