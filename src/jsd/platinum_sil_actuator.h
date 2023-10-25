#ifndef FASTCAT_PLATINUM_SIL_ACTUATOR_H_
#define FASTCAT_PLATINUM_SIL_ACTUATOR_H_

// Include related header (for cc files)

// Include C then C++ libraries

// Include external then project includes
#include "fastcat/jsd/actuator_manager_interface.h"
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_epd_sil_pub.h"

// TODO(dloret): Consider whether GetParams is needed (probably not right now)

namespace fastcat
{
typedef enum {
  PLATINUM_SIL_ACTUATOR_SMS_FAULTED,
  PLATINUM_SIL_ACTUATOR_SMS_HALTED,
  PLATINUM_SIL_ACTUATOR_SMS_MOTION,
} PlatinumSilActuatorStateMachineState;

typedef enum {
  PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_OKAY = 0,
  // Faults that can occur in handling of new commands
  PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION,
  // Faults that can occur in processing of Actuator's state machine
  PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_STO_ENGAGED,
  PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_INVALID_ELMO_SMS_DURING_MOTION,
} PlatinumSilActuatorFastcatFault;

class PlatinumSilActuator : public JsdDeviceBase,
                            public ActuatorManagerInterface
{
 public:
  PlatinumSilActuator();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
  void      Fault() override;
  void      Reset() override;
  double    GetActualPosition() override;
  bool      SetOutputPosition(double position) override;
  bool      HasAbsoluteEncoder() override;

  static std::string FastcatFaultToString(
      PlatinumSilActuatorFastcatFault fault);

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);
  void TransitionToState(PlatinumSilActuatorStateMachineState sms);
  static std::string StateMachineStateToString(
      PlatinumSilActuatorStateMachineState sms);

  jsd_slave_config_t                   jsd_slave_config_  = {};
  jsd_epd_sil_state_t                  jsd_epd_sil_state_ = {};
  PlatinumSilActuatorStateMachineState actuator_sms_;

 private:
  double  CntsToEu(int64_t cnts);
  double  EuToCnts(double eu);
  double  PosCntsToEu(int64_t cnts);
  int32_t PosEuToCnts(double eu);

  FaultType ProcessFaulted();
  FaultType ProcessHalted();
  FaultType ProcessMotion();

  bool HandleNewHaltCmd();
  bool HandleNewSetOutputPositionCmd(const DeviceCmd& cmd);
  bool HandleNewSetUnitModeCmd(const DeviceCmd& cmd);

  bool IsIdleFaultConditionMet();
  bool IsMotionFaultConditionMet();

  jsd_elmo_state_machine_state_t GetElmoStateMachineState()
  {
    return static_cast<jsd_elmo_state_machine_state_t>(
        state_->platinum_sil_actuator_state.elmo_state_machine_state);
  };

  double                          overall_reduction_         = 1.0;
  bool                            actuator_absolute_encoder_ = false;
  int32_t                         elmo_pos_offset_cnts_      = 1;
  PlatinumSilActuatorFastcatFault fastcat_fault_ =
      PLATINUM_SIL_ACTUATOR_FASTCAT_FAULT_OKAY;
};

}  // namespace fastcat

#endif