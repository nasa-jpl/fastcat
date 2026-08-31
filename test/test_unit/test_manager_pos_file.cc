// Tests for the Manager's saved-actuator-position file: when it is written,
// when it is invalidated, and which topologies own it at all.
//
// These are integration-flavored tests -- they drive a real Manager over an
// offline bus and assert on what lands on disk -- because the behavior under
// test is precisely the interaction between the brake state machine, the
// background writer thread, and the filesystem.
//
// Time is supplied externally (Manager::Process(external_time)) so the settling
// debounce and the actuator's own profile timing advance deterministically
// rather than at wall-clock speed. The fake clock is seeded from real time so it
// only ever moves forward relative to the Process() calls InitHardware() makes
// internally.

#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "fastcat/manager.h"
#include "jsd/jsd_time.h"

namespace
{
constexpr double kLoopRateHz   = 1000.0;
constexpr double kDt           = 1.0 / kLoopRateHz;
constexpr double kSettleSec    = 0.5;
constexpr double kStartPosEu   = 1.5;
constexpr double kTargetPosEu  = 0.5;
// Real-time budget for the background writer thread to service a request.
constexpr double kWriterTimeoutSec = 5.0;
// Real-time grace period granted to the writer before asserting that a file has
// NOT been written, so the assertion cannot pass merely because the writer had
// not been scheduled yet.
constexpr int kNegativeAssertGraceMs = 150;

std::string PosDirFor(const std::string& test_name)
{
  std::ostringstream ss;
  ss << "/tmp/fastcat_pos_file_test_" << getpid() << "_" << test_name;
  return ss.str();
}

bool FileExists(const std::string& path)
{
  struct stat st;
  return 0 == stat(path.c_str(), &st);
}

std::string ReadFile(const std::string& path)
{
  std::ifstream     f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Poll until `path` reaches the desired existence state, or the timeout expires.
// Returns true if the desired state was observed. Needed because saves posted
// from Process() are serviced asynchronously by the writer thread.
bool WaitForFileState(const std::string& path, bool should_exist)
{
  const double deadline = jsd_time_get_time_sec() + kWriterTimeoutSec;
  while (jsd_time_get_time_sec() < deadline) {
    if (FileExists(path) == should_exist) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return FileExists(path) == should_exist;
}

// A GoldActuator on an offline bus, with the position-file knobs parameterized.
std::string ActuatorTopologyYaml(const std::string& pos_dir, double settle_sec)
{
  std::ostringstream ss;
  ss << "fastcat:\n"
     << "  target_loop_rate_hz:                " << kLoopRateHz << "\n"
     << "  zero_latency_required:              False\n"
     << "  actuator_position_directory:        " << pos_dir << "\n"
     << "  actuator_fault_on_missing_pos_file: False\n"
     << "  actuator_position_save_settle_sec:  " << settle_sec << "\n"
     << "buses:\n"
     << "  - type: offline_bus\n"
     << "    ifname: eth0\n"
     << "    enable_autorecovery: False\n"
     << "    devices:\n"
     << "    - device_class:                   GoldActuator\n"
     << "      name:                           act_1\n"
     << "      actuator_type:                  revolute\n"
     << "      gear_ratio:                     100\n"
     << "      counts_per_rev:                 500\n"
     << "      max_speed_eu_per_sec:           10\n"
     << "      max_accel_eu_per_sec2:          30\n"
     << "      over_speed_multiplier:          2\n"
     << "      vel_tracking_error_eu_per_sec:  1000\n"
     << "      pos_tracking_error_eu:          1000\n"
     << "      peak_current_limit_amps:        10\n"
     << "      peak_current_time_sec:          3.0\n"
     << "      continuous_current_limit_amps:  5\n"
     << "      torque_slope_amps_per_sec:      0.5\n"
     << "      low_pos_cal_limit_eu:           -3.2\n"
     << "      low_pos_cmd_limit_eu:           -3.14159\n"
     << "      high_pos_cmd_limit_eu:          3.14159\n"
     << "      high_pos_cal_limit_eu:          3.2\n"
     // Short so the post-move HOLDING state expires quickly in fake time.
     << "      holding_duration_sec:           0.05\n"
     << "      elmo_brake_engage_msec:         10\n"
     << "      elmo_brake_disengage_msec:      20\n"
     << "      elmo_crc:                       12345\n"
     << "      elmo_drive_max_current_limit:   10\n"
     << "      smooth_factor:                  0\n";
  return ss.str();
}

// A topology with no actuators at all, pointed at the same position directory.
// LoadActuatorPosFile() bypasses the position file for this case, so nothing
// here may read, write, or delete it.
std::string NoActuatorTopologyYaml(const std::string& pos_dir)
{
  std::ostringstream ss;
  ss << "fastcat:\n"
     << "  target_loop_rate_hz:                " << kLoopRateHz << "\n"
     << "  zero_latency_required:              False\n"
     << "  actuator_position_directory:        " << pos_dir << "\n"
     << "  actuator_fault_on_missing_pos_file: False\n"
     << "buses:\n"
     << "  - type: fastcat_bus\n"
     << "    ifname: fastcat\n"
     << "    devices:\n"
     << "    - device_class: SignalGenerator\n"
     << "      name: sig_gen_1\n"
     << "      signal_generator_type: SINE_WAVE\n"
     << "      angular_frequency: 1.0\n"
     << "      phase: 0\n"
     << "      amplitude: 1.0\n"
     << "      offset: 0\n";
  return ss.str();
}

}  // namespace

namespace fastcat
{

class PosFileTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    pos_dir_ = PosDirFor(
        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    RemoveDir();
    ASSERT_EQ(0, mkdir(pos_dir_.c_str(), 0777));

    pos_file_      = pos_dir_ + "/fastcat_saved_positions.yaml";
    prev_pos_file_ = pos_dir_ + "/fastcat_saved_positions_prev.yaml";
    tmp_pos_file_  = pos_dir_ + "/fastcat_saved_positions.yaml.tmp";
  }

  void TearDown() override
  {
    // Destroy the manager first: its destructor joins the writer thread, so no
    // further writes can land in the directory we are about to delete.
    manager_.reset();
    RemoveDir();
  }

  void RemoveDir()
  {
    remove((pos_dir_ + "/fastcat_saved_positions.yaml").c_str());
    remove((pos_dir_ + "/fastcat_saved_positions_prev.yaml").c_str());
    remove((pos_dir_ + "/fastcat_saved_positions.yaml.tmp").c_str());
    rmdir(pos_dir_.c_str());
  }

  void SeedPosFile(double position)
  {
    std::ofstream f(pos_file_);
    ASSERT_TRUE(f.good());
    f << "actuators:\n";
    f << "  - actuator_name: act_1\n";
    f << "    position: " << position << "\n";
  }

  // Bring up a Manager on `yaml`. The fake clock is seeded from real time so
  // that it stays ahead of the Process() calls InitHardware() makes with the
  // default (real-time) argument.
  void StartManager(const std::string& yaml)
  {
    manager_ = std::make_unique<Manager>();
    t_       = jsd_time_get_time_sec();
    YAML::Node node = YAML::Load(yaml);
    ASSERT_TRUE(manager_->ConfigFromYaml(node, t_));
    // ConfigFromYaml() runs InitHardware() internally, which itself calls
    // Process() twice using real time; advance past that before taking over.
    t_ = jsd_time_get_time_sec() + kDt;
  }

  void Pump(int cycles)
  {
    for (int i = 0; i < cycles; ++i) {
      t_ += kDt;
      manager_->Process(t_);
    }
  }

  // Advance the fake clock by `seconds` worth of cycles.
  void PumpFor(double seconds) { Pump(static_cast<int>(seconds / kDt)); }

  void CommandMoveTo(double target)
  {
    DeviceCmd cmd{};
    cmd.name                                    = "act_1";
    cmd.type                                    = ACTUATOR_PROF_POS_CMD;
    cmd.actuator_prof_pos_cmd.target_position    = target;
    cmd.actuator_prof_pos_cmd.profile_velocity   = 1.0;
    cmd.actuator_prof_pos_cmd.end_velocity       = 0.0;
    cmd.actuator_prof_pos_cmd.profile_accel      = 5.0;
    cmd.actuator_prof_pos_cmd.relative           = 0;
    manager_->QueueCommand(cmd);
  }

  uint8_t MotorOn()
  {
    for (const auto& state : manager_->GetDeviceStates()) {
      if (state.type == GOLD_ACTUATOR_STATE) {
        return state.gold_actuator_state.motor_on;
      }
    }
    ADD_FAILURE() << "no gold actuator in topology";
    return 0;
  }

  double ActualPosition()
  {
    for (const auto& state : manager_->GetDeviceStates()) {
      if (state.type == GOLD_ACTUATOR_STATE) {
        return state.gold_actuator_state.actual_position;
      }
    }
    ADD_FAILURE() << "no gold actuator in topology";
    return 0.0;
  }

  // Pump until the drive reports the brakes engaged (motor power removed), or
  // give up after `limit_sec` of fake time.
  bool PumpUntilBrakesEngaged(double limit_sec)
  {
    const int limit_cycles = static_cast<int>(limit_sec / kDt);
    for (int i = 0; i < limit_cycles; ++i) {
      Pump(1);
      if (MotorOn() == 0) {
        return true;
      }
    }
    return false;
  }

  // Pump until the drive reports motor power applied (brakes releasing).
  bool PumpUntilMotorOn(double limit_sec)
  {
    const int limit_cycles = static_cast<int>(limit_sec / kDt);
    for (int i = 0; i < limit_cycles; ++i) {
      Pump(1);
      if (MotorOn() != 0) {
        return true;
      }
    }
    return false;
  }

  double SavedPosition()
  {
    YAML::Node node = YAML::Load(ReadFile(pos_file_));
    return node["actuators"][0]["position"].as<double>();
  }

  std::string SavedActuatorName()
  {
    YAML::Node node = YAML::Load(ReadFile(pos_file_));
    return node["actuators"][0]["actuator_name"].as<std::string>();
  }

  // Poll the position file until it holds `expected` (within `tol`), checking
  // every observation along the way for completeness.
  //
  // Existence is not a usable signal for "this cycle's save has landed": a fresh
  // write supersedes a pending invalidate in the writer mailbox, so the document
  // the previous cycle wrote can stay on disk continuously across a move. Waiting
  // on existence and then reading therefore races, and reads the stale position.
  // Waiting on the content is what the caller actually means.
  //
  // Empty reads are skipped rather than failed: the file is legitimately absent
  // during the invalidate window, and stat-then-read cannot distinguish absent
  // from truncated without a race of its own. Truncation is still caught -- a
  // partial document either fails to parse or is missing its fields below.
  bool WaitForSavedPosition(double expected, double tol)
  {
    const double deadline = jsd_time_get_time_sec() + kWriterTimeoutSec;
    while (jsd_time_get_time_sec() < deadline) {
      const std::string contents = ReadFile(pos_file_);
      if (!contents.empty()) {
        YAML::Node node;
        try {
          node = YAML::Load(contents);
        } catch (const YAML::Exception& e) {
          ADD_FAILURE() << "observed an unparseable position file: " << e.what()
                        << "\ncontents:\n"
                        << contents;
          return false;
        }
        if (!node["actuators"] || node["actuators"].size() != 1 ||
            !node["actuators"][0]["actuator_name"] ||
            !node["actuators"][0]["position"]) {
          ADD_FAILURE() << "observed an incomplete position file:\n" << contents;
          return false;
        }
        EXPECT_EQ("act_1",
                  node["actuators"][0]["actuator_name"].as<std::string>());
        if (std::fabs(node["actuators"][0]["position"].as<double>() - expected) <=
            tol) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  std::unique_ptr<Manager> manager_;
  std::string              pos_dir_;
  std::string              pos_file_;
  std::string              prev_pos_file_;
  std::string              tmp_pos_file_;
  double                   t_ = 0.0;
};

// The startup position is loaded from the file, and a bus that comes up already
// stopped must not rewrite it -- there is nothing new to persist.
TEST_F(PosFileTest, LoadsStartupPositionAndDoesNotResaveWhenNeverMoved)
{
  SeedPosFile(kStartPosEu);
  const std::string original = ReadFile(pos_file_);

  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));

  // SetActuatorPositions() runs after InitHardware()'s internal Process() calls,
  // so the applied encoder offset is not visible in published state until the
  // next Read(). One cycle is enough.
  Pump(1);

  // Position came from the file (within one encoder count).
  EXPECT_NEAR(kStartPosEu, ActualPosition(), 1e-3);

  // Idle well past the settling window; the file must be byte-identical and no
  // backup should have been created.
  PumpFor(3.0 * kSettleSec);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(kNegativeAssertGraceMs));

  EXPECT_TRUE(FileExists(pos_file_));
  EXPECT_EQ(original, ReadFile(pos_file_));
  EXPECT_FALSE(FileExists(prev_pos_file_));
}

// The core cycle: motion invalidates the file, and it is rewritten with the new
// position once the arm settles.
TEST_F(PosFileTest, InvalidatesOnMotionAndSavesAfterSettling)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));

  CommandMoveTo(kTargetPosEu);

  // Motor power applied -> the on-disk position is now untrustworthy and must be
  // removed before the arm can move anywhere.
  ASSERT_TRUE(PumpUntilMotorOn(1.0));
  EXPECT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/false))
      << "position file must be invalidated as soon as motors are powered";

  // Run the move to completion, through HOLDING and into HALTED.
  ASSERT_TRUE(PumpUntilBrakesEngaged(10.0));

  // Settle, then the save should land.
  PumpFor(2.0 * kSettleSec);
  ASSERT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/true))
      << "position file must be rewritten after the arm settles";

  EXPECT_EQ("act_1", SavedActuatorName());
  EXPECT_NEAR(kTargetPosEu, SavedPosition(), 1e-2);

  // Atomic-write bookkeeping: the temp file must not linger.
  EXPECT_FALSE(FileExists(tmp_pos_file_));

  // No _prev backup here, and that is expected: the backup is a copy of the
  // canonical file made at write time, and the canonical file had already been
  // removed by the invalidate-on-motion step. This is the normal path, so
  // _prev.yaml is effectively never produced during ordinary operation -- see
  // ShutdownOverExistingFileCreatesBackup for the case that does produce it.
  EXPECT_FALSE(FileExists(prev_pos_file_));
}

// The _prev backup is only produced when a save overwrites a canonical file that
// is still present -- i.e. a save that was not preceded by an invalidation.
TEST_F(PosFileTest, ShutdownOverExistingFileCreatesBackup)
{
  SeedPosFile(kStartPosEu);
  const std::string seeded = ReadFile(pos_file_);

  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));
  Pump(10);  // never moves, so the file is never invalidated

  manager_->Shutdown();

  ASSERT_TRUE(FileExists(prev_pos_file_));
  EXPECT_EQ(seeded, ReadFile(prev_pos_file_))
      << "backup must hold the contents the save replaced";
  EXPECT_NEAR(kStartPosEu, SavedPosition(), 1e-3);
  EXPECT_FALSE(FileExists(tmp_pos_file_));
}

// Regression test for the settling debounce. On an STO/e-stop/fault, drive power
// is cut while the joint is still moving and the joint coasts to rest against
// the brake -- so a save must NOT happen on the first brakes-engaged cycle.
TEST_F(PosFileTest, SaveIsDeferredUntilSettlingWindowElapses)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));

  CommandMoveTo(kTargetPosEu);
  ASSERT_TRUE(PumpUntilMotorOn(1.0));
  ASSERT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/false));

  // The instant the brakes engage the position is not yet trustworthy.
  ASSERT_TRUE(PumpUntilBrakesEngaged(10.0));
  ASSERT_EQ(0u, MotorOn());

  // Advance to just short of the settling window: still no file.
  PumpFor(0.8 * kSettleSec);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(kNegativeAssertGraceMs));
  EXPECT_FALSE(FileExists(pos_file_))
      << "save must be deferred until the brakes have been engaged for "
         "actuator_position_save_settle_sec";

  // Cross the threshold: now it saves.
  PumpFor(0.4 * kSettleSec);
  EXPECT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/true))
      << "save must happen once the settling window has elapsed";
}

// settle_sec == 0 restores the pre-debounce behavior, on the first
// brakes-engaged cycle. Documents the escape hatch.
TEST_F(PosFileTest, ZeroSettleSecSavesOnFirstBrakesEngagedCycle)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, 0.0));

  CommandMoveTo(kTargetPosEu);
  ASSERT_TRUE(PumpUntilMotorOn(1.0));
  ASSERT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/false));

  ASSERT_TRUE(PumpUntilBrakesEngaged(10.0));
  // No additional pumping: the save must already have been posted.
  EXPECT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/true));
}

// Only one save per motion->stop cycle: a long idle must not keep rewriting.
TEST_F(PosFileTest, SavesOncePerMotionCycle)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));

  CommandMoveTo(kTargetPosEu);
  ASSERT_TRUE(PumpUntilMotorOn(1.0));
  ASSERT_TRUE(PumpUntilBrakesEngaged(10.0));
  PumpFor(2.0 * kSettleSec);
  // Wait for the new position rather than mere existence: the seeded file is
  // still on disk until the writer services this cycle's request, so a snapshot
  // taken on existence alone can capture the seed and then differ from the real
  // save below.
  ASSERT_TRUE(WaitForSavedPosition(kTargetPosEu, 1e-2));

  const std::string first_contents = ReadFile(pos_file_);

  // Idle for many more settling windows.
  PumpFor(10.0 * kSettleSec);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(kNegativeAssertGraceMs));

  EXPECT_EQ(first_contents, ReadFile(pos_file_));
}

// Shutdown while stopped is the clean path: positions are persisted, and the
// wait is synchronous so the file is on disk by the time Shutdown() returns.
TEST_F(PosFileTest, ShutdownWhileStoppedSavesSynchronously)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));

  // Move, settle, then invalidate again by starting a second move and letting it
  // finish, so there is something new to write at shutdown.
  CommandMoveTo(kTargetPosEu);
  ASSERT_TRUE(PumpUntilMotorOn(1.0));
  ASSERT_TRUE(PumpUntilBrakesEngaged(10.0));
  PumpFor(2.0 * kSettleSec);
  ASSERT_TRUE(WaitForFileState(pos_file_, /*should_exist=*/true));

  remove(pos_file_.c_str());
  ASSERT_FALSE(FileExists(pos_file_));

  manager_->Shutdown();

  // No WaitForFileState(): Shutdown() must block until the write is durable.
  EXPECT_TRUE(FileExists(pos_file_))
      << "Shutdown() must not return before the position file is on disk";
  EXPECT_NEAR(kTargetPosEu, SavedPosition(), 1e-2);
}

// Shutdown mid-motion must leave NO file rather than an in-motion position: the
// next startup is expected to fault rather than trust a bad position. This is a
// deliberate policy choice -- see actuator_fault_on_missing_pos_file.
TEST_F(PosFileTest, ShutdownWhileMovingInvalidatesRatherThanSaving)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, kSettleSec));

  CommandMoveTo(kTargetPosEu);
  ASSERT_TRUE(PumpUntilMotorOn(1.0));
  ASSERT_NE(0u, MotorOn());

  manager_->Shutdown();

  EXPECT_FALSE(FileExists(pos_file_))
      << "an in-motion shutdown must leave the position file absent, not write "
         "a mid-travel position";
}

// A topology with no actuators bypasses the position file entirely, so it must
// not delete a file that belongs to some other topology sharing the directory.
TEST_F(PosFileTest, NoActuatorTopologyLeavesPositionFileUntouched)
{
  SeedPosFile(kStartPosEu);
  const std::string original = ReadFile(pos_file_);

  StartManager(NoActuatorTopologyYaml(pos_dir_));
  PumpFor(3.0 * kSettleSec);
  manager_->Shutdown();
  std::this_thread::sleep_for(
      std::chrono::milliseconds(kNegativeAssertGraceMs));

  ASSERT_TRUE(FileExists(pos_file_))
      << "a topology that bypasses the position file must not delete it";
  EXPECT_EQ(original, ReadFile(pos_file_));
}

// A negative settling window is meaningless and must be rejected at
// configuration time rather than silently accepted.
TEST_F(PosFileTest, NegativeSettleSecIsRejected)
{
  SeedPosFile(kStartPosEu);
  manager_        = std::make_unique<Manager>();
  YAML::Node node = YAML::Load(ActuatorTopologyYaml(pos_dir_, -1.0));
  EXPECT_FALSE(manager_->ConfigFromYaml(node, jsd_time_get_time_sec()));
}

// The canonical file is replaced by an atomic rename, so a reader never observes
// a partial write: every observation is either the old or the new full document.
TEST_F(PosFileTest, PositionFileIsAlwaysCompleteAndParseable)
{
  SeedPosFile(kStartPosEu);
  StartManager(ActuatorTopologyYaml(pos_dir_, 0.0));

  for (int cycle = 0; cycle < 3; ++cycle) {
    const double target = (cycle % 2 == 0) ? kTargetPosEu : kStartPosEu;
    CommandMoveTo(target);
    ASSERT_TRUE(PumpUntilMotorOn(1.0));
    ASSERT_TRUE(PumpUntilBrakesEngaged(10.0));

    // Whenever the file is present it must be a complete, parseable document
    // with a usable position -- never truncated. WaitForSavedPosition() checks
    // that of every observation it makes while waiting for this cycle's save.
    ASSERT_TRUE(WaitForSavedPosition(target, 1e-2))
        << "cycle " << cycle << ": save of " << target
        << " never landed; file holds:\n"
        << ReadFile(pos_file_);

    EXPECT_FALSE(FileExists(tmp_pos_file_))
        << "temp file must not survive a completed write";
  }
}

}  // namespace fastcat
