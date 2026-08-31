#ifndef FASTCAT_MANAGER_H_
#define FASTCAT_MANAGER_H_

// Include related header (for cc files)

// Include c then c++ libraries
#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Include external then project includes
#include <yaml-cpp/yaml.h>

#include "fastcat/thread_safe_queue.h"
#include "fastcat/device_base.h"
#include "fastcat/jsd/actuator.h"
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd.h"

namespace fastcat
{
typedef std::pair<std::string, std::shared_ptr<DeviceBase>> DevicePair;
typedef std::pair<std::string, jsd_t*>                      JSDPair;

/** @brief Fastcat::Manager is the main application interface to manage all
 * fastcat devices
 */
class Manager
{
 public:
  Manager();
  ~Manager();

  /** @brief Shutdown the bus and joins all threads.
   *
   * Also Writes Actuator positions to file if applicable.
   * @return void
   */
  void Shutdown();

  /** @brief Capture current actuator positions and write them to file.
   *
   * Intended for callers that want to persist positions without tearing down
   * the bus. The write itself is crash-safe (atomic rename), and the positions
   * are snapshotted under the same internal mutex used by Process(), so this is
   * safe to call from any thread while the process loop is still running. The
   * snapshot is taken under that mutex but the disk write is not, so the RT loop
   * is not blocked on I/O.
   *
   * Must NOT be called from a signal handler: it takes a mutex and allocates,
   * neither of which is async-signal-safe, and if the signal is delivered on the
   * thread already inside Process() it will self-deadlock on that mutex. Set a
   * flag in the handler and call this from your normal control flow instead.
   *
   * No-op for topologies that do not persist positions (no actuators, or only
   * actuators with absolute encoders).
   * @return void
   */
  void SaveActuatorPositions();

  /** @brief Method that accepts a fastcat topology yaml and intializes bus
   *
   *  @return true on successful initialization. If false, application should
   * quit.
   */
  bool ConfigFromYaml(const YAML::Node& node, double external_time = -1);

  /** @brief Parses YAML configuration and creates device objects (no hardware init)
   *
   *  @return true on successful configuration. If false, application should quit.
   */
  bool CreateConfigFromYaml(const YAML::Node& node, double external_time = -1);

  /** @brief Initializes EtherCAT hardware (executes deferred jsd_init calls)
   *
   *  @return true on successful hardware initialization. If false, application should quit.
   */
  bool InitHardware();

  /** @brief Updates synchronous PDO and background async SDO requests.
   *
   *  Process() proceeds by
   *  1. Triggers PDO Read on EtherCAT bus
   *  2. Reads all JSD devices into the manager
   *  3. Reads all Fastcat devices (including observed device state propagation)
   *  4. Calls DeviceBase::Process() on Fastcat Devices, checking for faults and
   * SDO requests
   *  5. Writes Device Commands (Includes User commands and Fastcat Device
   * Commands)
   *  6. Calls Process() on JSD devices, checking for faults
   *  7. Triggers PDO Write on EtherCAT bus
   *
   *   Note: For best performance, the application should call the
   * Manager::Process() function at the same frequency as the input YAML field
   * `target_loop_rate_hz`. This parameter is needed by certain devices for
   * profiling and filtering.
   *
   *   @param external_time Supply an external time if desired, otherwise
   *          defaults to jsd supplied system time
   *   @return Return true if bus is not faulted, otherwise a bus fault is
   * active.
   */
  bool Process(double external_time = -1);

  /** @brief Interface to command devices on the bus
   *
   *  Commands each have a name field and that name is used to dispatch the
   * command to the right device. If the provided name is not found on the
   * loaded bus topology, a warning message is printed to stdout but no faults
   * are triggered.
   *
   *  @return void
   */
  void QueueCommand(DeviceCmd& cmd);

  /** @brief Returns list of device states
   *
   *  @return device states
   */
  std::vector<DeviceState> GetDeviceStates();

  /** @brief Returns list of device state pointers
   *
   *  Provided for potential optimization. GetDeviceStates() generally performs
   *  better. This function is not intended to be called in a realtime process loop
   *  because it dynamically allocates memory to hold a vector of device state pointers.
   *  @return device states
   */
  std::vector<std::shared_ptr<const DeviceState>> GetDeviceStatePointers();

  /** @brief Public getter to the YAML `target_loop_rate_hz` parameter
   *  @return loop rate in hz
   */
  double GetTargetLoopRate();

  /** @brief Public getter retrieve fault status
   *  @return true if bus is faulted
   */
  bool IsFaulted();

  /** @brief Attempts to recover a faulty JSD bus by name
   *
   *  A bus fault example is spurious Working Counter (WKC) error, or perhaps an
   * intended power cycle of a configured EtherCAT slave.
   *
   *  @param ifname the name of the JSD bus configured in the input topology
   * YAML
   *  @return true if bus ifname exists, does not indicate the error is fixed.
   */
  bool RecoverBus(std::string ifname);

  /** @brief Triggers a single device to reset
   *
   *  @param device_name the name of the device to reset, calls its Reset()
   * method
   *  @return true if the device exists, false if device_name is invalid.
   */
  bool ExecuteDeviceReset(std::string device_name);

  /** @brief Triggers a single device to fault
   *
   *  @param device_name the name of the device to fault, calls its Fault()
   * method
   *  @return true if the device exists, false if device_name is invalid.
   */
  bool ExecuteDeviceFault(std::string device_name);

  /** @brief Triggers all devices to Reset
   *
   *   This function loops over all devices in the topology and calls their
   * DeviceBase::Reset() Method.
   *
   *   @return void
   */
  void ExecuteAllDeviceResets();

  /** @brief Triggers all devices to Fault
   *
   *   This function loops over all devices in the topology and calls their
   * DeviceBase::Fault() Method. An example usage is a soft-stop GUI button that
   * can be used to arrest motion.
   *
   *   @return void
   */
  void ExecuteAllDeviceFaults();

  /** @brief checks if the SdoResponse Queue is empty
   *  @return if queue is empty
   */
  bool IsSdoResponseQueueEmpty();

  /** @brief get the result of a background SDO operation
   *
   *  If the SDO Response queue contains any responses, this function pops the
   *  oldest value and returns it to the application.
   *
   *  @return true if the return reference 'res' is valid
   */
  bool PopSdoResponseQueue(SdoResponse& res);

  /** @brief get actuator parameters
   *
   *  @return true if the return reference 'parameters' is valid
   */
  bool GetActuatorParams(const std::string&                 name,
                         fastcat::Actuator::ActuatorParams& param);

  /** @brief names of actuator devices
   */
  void GetDeviceNamesByType(std::vector<std::string>&,
                            fastcat::DeviceStateType);

  /** @brief Set interpolation algorithm to use 3rd order cubic interpolation between
   * knot points for all actuators */
  void SetExplicitInterpolationAlgorithmCubic();

  /** @brief Set interpolation algorithm to use 1st order linear interpolation between
   * knot points for both position and velocity for all actuators */
  void SetExplicitInterpolationAlgorithmLinear();

  /** @brief Set interpolation algorithm to use the timestamp in the CSP message
   * generated by the calling module for determining time of interpolation knot-points,
   * which accounts for latency in message passing */
  void SetExplicitInterpolationTimestampSourceCspMessage();

  /** @brief Set interpolation algorithm to use the timestamp when the CSP message was
   * received according to fastcat's clock; this mode of interpolation does not account
   * for latency in message passing */
  void SetExplicitInterpolationTimestampSourceClock();

  /** @brief Set number of cycles of the calling module to delay the onset of explicit
   * interpolation, e.g. if delay is set to 3, fastcast will wait until 4 messages 
   * accumulate in the buffer before beginning motion profile. A larger buffer provides 
   * a greater margin against motion setpoints not being received fast enough as motion
   * profile is executing, which may happen due to process jitter or message passing
   * latency */
  bool SetExplicitInterpolationCyclesDelay(size_t delay);

  /** @brief CSP interpolation will transition to a holding state if it has not received
   * a CSP message within the provided number of fastcat process cycles */
  bool SetInterpolationCyclesStale(size_t cycles);


 private:
  bool ConfigJSDBusFromYaml(const YAML::Node& node, double external_time);
  bool ConfigFastcatBusFromYaml(const YAML::Node& node, double external_time);
  bool ConfigOfflineBusFromYaml(const YAML::Node& node, double external_time);
  bool WriteCommands();
  bool ConfigSignals();
  bool SortFastcatDevice(
      std::shared_ptr<DeviceBase>               device,
      std::vector<std::shared_ptr<DeviceBase>>& sorted_devices,
      std::vector<std::string>                  parents);

  bool LoadActuatorPosFile();
  bool ValidateActuatorPosFile();
  // Full path of the saved positions file, so every message about it can name
  // the file the operator has to go look at.
  std::string PosFilePath() const;
  bool SetActuatorPositions();
  void GetActuatorPositions();
  // Serialize actuator_pos_map_ to a YAML string. Cheap, pure CPU; called on
  // the RT thread under parameter_mutex_ so it sees a consistent snapshot.
  std::string BuildActuatorPosYaml();
  // Perform the actual disk write (temp file + fsync + _prev backup + atomic
  // rename) for the already-serialized `contents`. Runs ONLY on the background
  // writer thread; touches no shared device state, takes no RT lock.
  void WritePosFileToDisk(const std::string& contents);
  void InvalidateActuatorPosFile();
  // fsync the position directory so the last rename/unlink is durable across a
  // power loss. Runs ONLY on the background writer thread.
  void SyncPosFileDirectory();
  // Background position-file writer: keeps all disk I/O (fsync, rename, backup
  // copy) off the RT Process() thread so a save cannot cause a cycle slip.
  void StartPosWriter();
  void StopPosWriter();
  void PosWriterLoop();
  // Post a write (contents) or an invalidate request to the writer thread. If
  // `wait` is true, block until the writer has processed it (used on shutdown
  // to guarantee durability before exit). Both return the request sequence
  // number, which can be handed to WaitForPosWriter() later to defer the block
  // until after a caller-held lock is released; 0 means the request was already
  // serviced inline because the writer thread is not running.
  uint64_t PostPosWriteRequest(std::string contents, bool wait);
  uint64_t PostPosInvalidateRequest(bool wait);
  // Block until the writer has drained request `seq`. Never call while holding
  // parameter_mutex_: the wait is fsync-bound and Process() contends on it.
  void WaitForPosWriter(uint64_t seq);
  // True iff every GOLD/PLATINUM actuator has its brake engaged (motor_on == 0,
  // i.e. unpowered and mechanically held). Actuators with absolute encoders are
  // ignored (their positions are not persisted). Returns false if there are no
  // relevant actuators.
  bool AllBrakesEngaged();
  // Called at the end of each Process() cycle (under parameter_mutex_). Once
  // AllBrakesEngaged() has held for pos_save_settle_sec_ it saves the current
  // positions; on the falling edge (motion starting) it immediately invalidates
  // the saved file so a stale in-motion position can never be loaded.
  void UpdatePositionFileOnBrakeState(double monotonic_time);
  bool CheckDeviceNameIsUnique(std::string name);
  struct JsdBusInitParams {
    std::string ifname;
    jsd_t*      jsd;
    bool        enable_autorecovery;
  };

  double                        target_loop_rate_hz_                = 0.0;
  bool                          zero_latency_required_              = true;
  bool                          faulted_                            = true;
  bool                          actuator_fault_on_missing_pos_file_ = true;
  bool                          online_devices_exist_               = false;
  std::string                   actuator_position_directory_;
  std::map<std::string, jsd_t*> jsd_map_;

  std::map<std::string, std::shared_ptr<DeviceBase>> device_map_;
  std::vector<std::shared_ptr<DeviceBase>>           fastcat_device_list_;
  std::vector<std::shared_ptr<JsdDeviceBase>>        jsd_device_list_;
  std::shared_ptr<ThreadSafeQueue<DeviceCmd>>             cmd_queue_;
  std::vector<DeviceState>                           states_;
  std::map<std::string, ActuatorPosData>             actuator_pos_map_;
  std::unordered_map<std::string, bool>              unique_device_map_;
  std::shared_ptr<std::queue<SdoResponse>>           sdo_response_queue_;

  std::vector<JsdBusInitParams> pending_jsd_inits_;

  std::mutex parameter_mutex_;

  // Falling-edge tracking for the invalidate-on-motion path. Starts true so a
  // bus that comes up already stopped is not treated as a transition.
  bool prev_all_brakes_engaged_ = true;

  // Set whenever the motors are powered (brakes not all engaged), cleared once a
  // save completes. Gating the save on this, rather than on an edge, guarantees
  // exactly one save per motion->stop cycle and means a bus that comes up already
  // stopped does not re-save the positions it just loaded.
  bool saw_motion_since_last_save_ = false;

  // monotonic_time at which the brakes most recently became fully engaged, or
  // -1.0 when they are not. Used to enforce the pos_save_settle_sec_ debounce.
  double brakes_engaged_since_ = -1.0;

  // How long all brakes must stay continuously engaged before positions are
  // considered settled and saved. Guards against persisting a mid-travel
  // position when drive power is cut at speed (STO/e-stop/fault) and the joint
  // coasts to rest against its brake. YAML: actuator_position_save_settle_sec.
  double pos_save_settle_sec_ = 0.5;

  // True only for topologies that actually persist actuator positions, i.e.
  // those with at least one non-absolute-encoder GOLD/PLATINUM actuator. Set by
  // LoadActuatorPosFile(), which bypasses all position-file handling otherwise.
  // Every save/invalidate path must check this: AllBrakesEngaged() reports false
  // when it finds no relevant actuator, which would otherwise be read as "in
  // motion" and delete a position file this topology does not own (it may be
  // shared with another topology that does use incremental encoders).
  bool pos_file_enabled_ = false;

  // ---- Background position-file writer ----
  // A single dedicated thread performs all disk I/O for the position file so
  // that no fsync/rename/backup-copy ever runs on the RT Process() thread. The
  // RT thread only serializes the (tiny) YAML string under parameter_mutex_ and
  // hands it off via the single-slot coalescing mailbox below.
  std::thread             pos_writer_thread_;
  std::mutex              pos_writer_mutex_;
  std::condition_variable pos_writer_cv_;        // RT -> writer: new request
  std::condition_variable pos_writer_done_cv_;   // writer -> RT: request drained
  std::string             pos_pending_contents_;  // payload for a pending write
  bool                    pos_pending_write_      = false;
  bool                    pos_pending_invalidate_ = false;
  bool                    pos_writer_stop_        = false;  // ask writer to exit
  // Monotonic counters to let a waiter (shutdown) know its request was handled.
  uint64_t                pos_request_seq_        = 0;  // incremented on each post
  uint64_t                pos_processed_seq_      = 0;  // writer sets = seq handled
  // Whether pos_writer_thread_ exists and will drain the mailbox. Atomic because
  // it is written by StartPosWriter()/StopPosWriter() on the application thread
  // but read by the RT Process() thread (via PostPos*Request) to decide between
  // handing off to the writer and writing inline.
  std::atomic<bool>       pos_writer_running_{false};

};
}  // namespace fastcat

#endif
