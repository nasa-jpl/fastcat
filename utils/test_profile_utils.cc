#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "profile_utils.h"

namespace
{
using profile_utils::PosProfileArgs;
using profile_utils::VelProfileArgs;

TEST(IsSafeElmoState, AcceptsEveryStateInTheSafeList)
{
  for (auto safe_state : profile_utils::kSafeElmoStates) {
    EXPECT_TRUE(profile_utils::IsSafeElmoState(
        static_cast<uint32_t>(safe_state)))
        << "0x" << std::hex << safe_state;
  }
}

TEST(IsSafeElmoState, RejectsFaultAndQuickStopStates)
{
  EXPECT_FALSE(profile_utils::IsSafeElmoState(
      JSD_ELMO_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON));
  EXPECT_FALSE(profile_utils::IsSafeElmoState(
      JSD_ELMO_STATE_MACHINE_STATE_OPERATION_ENABLED));
  EXPECT_FALSE(profile_utils::IsSafeElmoState(
      JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE));
  EXPECT_FALSE(profile_utils::IsSafeElmoState(
      JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE));
  EXPECT_FALSE(
      profile_utils::IsSafeElmoState(JSD_ELMO_STATE_MACHINE_STATE_FAULT));
  EXPECT_FALSE(profile_utils::IsSafeElmoState(profile_utils::kNoStateLogged));
}

TEST(TicksFromSeconds, TruncatesTowardsZero)
{
  EXPECT_EQ(128, profile_utils::TicksFromSeconds(2.0, 64.0));
  EXPECT_EQ(32, profile_utils::TicksFromSeconds(0.5, 64.0));
  // 0.3 * 64 = 19.2 ticks
  EXPECT_EQ(19, profile_utils::TicksFromSeconds(0.3, 64.0));
  EXPECT_EQ(0, profile_utils::TicksFromSeconds(0.0, 64.0));
}

const char* kTwoBusConfig = R"(
target_loop_rate_hz: 100
buses:
  - type: jsd_bus
    ifname: eth_ecat
    devices:
      - device_class: GoldActuator
        name: gold_act_1
      - device_class: GoldActuator
        name: gold_act_2
  - type: fastcat_bus
    devices:
      - device_class: SignalGenerator
        name: sig_gen_1
)";

TEST(CollectDeviceNames, ReturnsNamesFromEveryBusInOrder)
{
  std::vector<std::string> names =
      profile_utils::CollectDeviceNames(YAML::Load(kTwoBusConfig));
  ASSERT_EQ(3u, names.size());
  EXPECT_EQ("gold_act_1", names[0]);
  EXPECT_EQ("gold_act_2", names[1]);
  EXPECT_EQ("sig_gen_1", names[2]);
}

TEST(CollectDeviceNames, ToleratesMissingBusesAndDevicesKeys)
{
  EXPECT_TRUE(
      profile_utils::CollectDeviceNames(YAML::Load("target_loop_rate_hz: 100"))
          .empty());
  EXPECT_TRUE(profile_utils::CollectDeviceNames(
                  YAML::Load("buses:\n  - type: jsd_bus\n"))
                  .empty());
}

TEST(ValidateActuatorName, AcceptsNamePresentOnAnyBus)
{
  YAML::Node node = YAML::Load(kTwoBusConfig);
  EXPECT_TRUE(profile_utils::ValidateActuatorName(node, "gold_act_1"));
  EXPECT_TRUE(profile_utils::ValidateActuatorName(node, "sig_gen_1"));
}

TEST(ValidateActuatorName, RejectsUnknownName)
{
  YAML::Node node = YAML::Load(kTwoBusConfig);
  EXPECT_FALSE(profile_utils::ValidateActuatorName(node, "gold_act_3"));
  EXPECT_FALSE(profile_utils::ValidateActuatorName(node, ""));
}

TEST(MakeTelemetryFilename, EmbedsLocalTimestampAndTag)
{
  std::tm tm_local  = {};
  tm_local.tm_year  = 2026 - 1900;
  tm_local.tm_mon   = 8;  // September
  tm_local.tm_mday  = 2;
  tm_local.tm_hour  = 14;
  tm_local.tm_min   = 5;
  tm_local.tm_sec   = 6;
  tm_local.tm_isdst = -1;
  std::time_t when  = std::mktime(&tm_local);

  EXPECT_EQ("20260902_140506_pos_prof_telem.csv",
            profile_utils::MakeTelemetryFilename("pos_prof", when));
  EXPECT_EQ("20260902_140506_vel_prof_telem.csv",
            profile_utils::MakeTelemetryFilename("vel_prof", when));
}

TEST(TrapezoidalMoveDuration, TrapezoidalWhenMaxVelocityIsReached)
{
  // accel=2, max_vel=4 -> 2s ramps covering 4 rad each. A 20 rad move leaves
  // 12 rad of cruise at 4 rad/s = 3s, so 2 + 3 + 2 = 7s.
  EXPECT_NEAR(7.0, profile_utils::TrapezoidalMoveDuration(20.0, 2.0, 4.0),
              1e-9);
}

TEST(TrapezoidalMoveDuration, TriangularWhenMoveIsTooShortToReachMaxVelocity)
{
  // Same ramps need 8 rad to reach max_vel; a 4 rad move peaks early at
  // 2*sqrt(4/2) = 2.828s.
  EXPECT_NEAR(2.0 * std::sqrt(4.0 / 2.0),
              profile_utils::TrapezoidalMoveDuration(4.0, 2.0, 4.0), 1e-9);
}

TEST(TrapezoidalMoveDuration, IsContinuousAcrossTheProfileShapeBoundary)
{
  // At exactly 2 * accel_distance the two branches must agree.
  const double accel = 2.0, max_velocity = 4.0;
  const double boundary_distance =
      max_velocity * max_velocity / accel;  // 2 * 0.5 * v^2 / a
  EXPECT_NEAR(profile_utils::TrapezoidalMoveDuration(
                  boundary_distance - 1e-9, accel, max_velocity),
              profile_utils::TrapezoidalMoveDuration(boundary_distance, accel,
                                                     max_velocity),
              1e-6);
}

TEST(TrapezoidalMoveDuration, GrowsMonotonicallyWithDistance)
{
  double previous = 0.0;
  for (double distance = 0.5; distance < 30.0; distance += 0.5) {
    double duration = profile_utils::TrapezoidalMoveDuration(distance, 2.0, 4.0);
    EXPECT_GT(duration, previous) << "distance=" << distance;
    previous = duration;
  }
}

TEST(VelocityProfileDuration, SumsBothRampsAndTheCruiseHold)
{
  // accel=2, cruise=6 -> 3s ramp up + 5s hold + 3s ramp down
  EXPECT_NEAR(11.0, profile_utils::VelocityProfileDuration(2.0, 6.0, 5.0),
              1e-9);
}

TEST(VelocityProfileDuration, IsUnaffectedByCruiseSpeedSign)
{
  EXPECT_NEAR(profile_utils::VelocityProfileDuration(2.0, 6.0, 5.0),
              profile_utils::VelocityProfileDuration(2.0, -6.0, 5.0), 1e-9);
}

TEST(ParseDouble, AcceptsWellFormedNumbers)
{
  double value = 0.0;
  EXPECT_TRUE(profile_utils::ParseDouble("2.5", value));
  EXPECT_DOUBLE_EQ(2.5, value);
  EXPECT_TRUE(profile_utils::ParseDouble("-3", value));
  EXPECT_DOUBLE_EQ(-3.0, value);
  EXPECT_TRUE(profile_utils::ParseDouble("1e-3", value));
  EXPECT_DOUBLE_EQ(0.001, value);
}

TEST(ParseDouble, RejectsGarbageInsteadOfSilentlyYieldingZero)
{
  double value = 0.0;
  EXPECT_FALSE(profile_utils::ParseDouble("", value));
  EXPECT_FALSE(profile_utils::ParseDouble("abc", value));
  EXPECT_FALSE(profile_utils::ParseDouble("2.5rad", value));
  EXPECT_FALSE(profile_utils::ParseDouble("--2", value));
}

TEST(ParseVelProfileArgs, AcceptsNominalArguments)
{
  VelProfileArgs args;
  std::string    error;
  ASSERT_TRUE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "gold_act_1", "2.0", "-5.0", "3.0"}, args, error))
      << error;
  EXPECT_EQ("cfg.yaml", args.config_path);
  EXPECT_EQ("gold_act_1", args.actuator_name);
  EXPECT_DOUBLE_EQ(2.0, args.accel);
  EXPECT_DOUBLE_EQ(-5.0, args.cruise_speed);
  EXPECT_DOUBLE_EQ(3.0, args.cruise_duration);
}

TEST(ParseVelProfileArgs, RejectsWrongArgumentCount)
{
  VelProfileArgs args;
  std::string    error;
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs({}, args, error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "gold_act_1", "2.0", "5.0"}, args, error));
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "gold_act_1", "2.0", "5.0", "3.0", "extra"}, args, error));
}

TEST(ParseVelProfileArgs, RejectsOutOfRangeValues)
{
  VelProfileArgs args;
  std::string    error;
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "a", "0", "5.0", "3.0"}, args, error));
  EXPECT_EQ("acceleration must be positive", error);
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "a", "-1", "5.0", "3.0"}, args, error));
  EXPECT_EQ("acceleration must be positive", error);
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "a", "2.0", "0", "3.0"}, args, error));
  EXPECT_EQ("cruise_speed must be non-zero", error);
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "a", "2.0", "5.0", "0"}, args, error));
  EXPECT_EQ("cruise_duration must be positive", error);
}

TEST(ParseVelProfileArgs, ReportsWhichFieldWasNotANumber)
{
  VelProfileArgs args;
  std::string    error;
  EXPECT_FALSE(profile_utils::ParseVelProfileArgs(
      {"cfg.yaml", "a", "2.0", "fast", "3.0"}, args, error));
  EXPECT_NE(std::string::npos, error.find("cruise_speed")) << error;
}

TEST(ParsePosProfileArgs, AcceptsNominalArguments)
{
  PosProfileArgs args;
  std::string    error;
  ASSERT_TRUE(profile_utils::ParsePosProfileArgs(
      {"cfg.yaml", "gold_act_1", "2.0", "5.0", "-3.0"}, args, error))
      << error;
  EXPECT_EQ("cfg.yaml", args.config_path);
  EXPECT_EQ("gold_act_1", args.actuator_name);
  EXPECT_DOUBLE_EQ(2.0, args.accel);
  EXPECT_DOUBLE_EQ(5.0, args.max_velocity);
  EXPECT_DOUBLE_EQ(-3.0, args.relative_position);
}

fastcat::DeviceState MakeDeviceState(const std::string&    name,
                                    fastcat::DeviceStateType type)
{
  fastcat::DeviceState state;
  state.name = name;
  state.type = type;
  return state;
}

TEST(FindGoldActuatorState, ReturnsTheMatchingGoldActuator)
{
  std::vector<fastcat::DeviceState> states = {
      MakeDeviceState("sig_gen_1", fastcat::SIGNAL_GENERATOR_STATE),
      MakeDeviceState("gold_act_1", fastcat::GOLD_ACTUATOR_STATE),
      MakeDeviceState("gold_act_2", fastcat::GOLD_ACTUATOR_STATE),
  };
  states[1].gold_actuator_state.actual_position = 1.25;

  const fastcat::GoldActuatorState* found =
      profile_utils::FindGoldActuatorState(states, "gold_act_1");
  ASSERT_NE(nullptr, found);
  EXPECT_DOUBLE_EQ(1.25, found->actual_position);
}

TEST(FindGoldActuatorState, ReturnsNullptrOnNameOrTypeMismatch)
{
  std::vector<fastcat::DeviceState> states = {
      MakeDeviceState("sig_gen_1", fastcat::SIGNAL_GENERATOR_STATE),
      MakeDeviceState("gold_act_1", fastcat::GOLD_ACTUATOR_STATE),
  };
  // Right name, wrong device class
  EXPECT_EQ(nullptr, profile_utils::FindGoldActuatorState(states, "sig_gen_1"));
  EXPECT_EQ(nullptr, profile_utils::FindGoldActuatorState(states, "no_such"));
  EXPECT_EQ(nullptr, profile_utils::FindGoldActuatorState({}, "gold_act_1"));
}

TEST(ParsePosProfileArgs, RejectsOutOfRangeValues)
{
  PosProfileArgs args;
  std::string    error;
  EXPECT_FALSE(profile_utils::ParsePosProfileArgs(
      {"cfg.yaml", "a", "0", "5.0", "3.0"}, args, error));
  EXPECT_EQ("acceleration must be positive", error);
  EXPECT_FALSE(profile_utils::ParsePosProfileArgs(
      {"cfg.yaml", "a", "2.0", "-5.0", "3.0"}, args, error));
  EXPECT_EQ("max_velocity must be positive", error);
  EXPECT_FALSE(profile_utils::ParsePosProfileArgs(
      {"cfg.yaml", "a", "2.0", "5.0", "0"}, args, error));
  EXPECT_EQ("relative_position cannot be zero", error);
}

}  // namespace
