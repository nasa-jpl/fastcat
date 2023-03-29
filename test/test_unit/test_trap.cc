#include <gtest/gtest.h>

#include "fastcat/trap.h"

namespace
{
class TrapTest : public ::testing::Test
{
 protected:
  void SetUp() override {}

  trap_t trap_1_;
};

TEST_F(TrapTest, TrapVelocityReInit)
{
  double time_initial     = 0.0;
  double position_initial = 0.0;
  double velocity_initial = 0.0;
  double velocity_final   = 1.0;
  double acceleration     = 1.0;
  double max_time         = 10.0;

  // Generate the first version of the trap
  trap_generate_vel(&trap_1_, time_initial, position_initial, velocity_initial,
                    velocity_final, acceleration, max_time);

  double dt = 0.01;

  double tracking_time     = time_initial;
  double tracking_position = position_initial;
  double tracking_velocity = velocity_initial;

  for (int i = 0; i < 1000; i++) {
    // Update trap before any change in time has occurred
    trap_update_vel(&trap_1_, tracking_time, &tracking_position,
                    &tracking_velocity);

    // Increment time
    tracking_time += dt;

    // Regenerate trap to simulate incoming updated setpoint
    trap_generate_vel(&trap_1_, tracking_time, tracking_position,
                      tracking_velocity, velocity_final, acceleration,
                      max_time);
  }

  // The minimum dt used in track vel should guarantee that
  // motion occurs despite regeneration happening every cycle
  EXPECT_TRUE(tracking_position > position_initial);
}

}  // namespace
