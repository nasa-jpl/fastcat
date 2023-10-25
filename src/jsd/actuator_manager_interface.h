#ifndef FASTCAT_ACTUATOR_MANAGER_INTERFACE_H_
#define FASTCAT_ACTUATOR_MANAGER_INTERFACE_H_

namespace fastcat
{
class ActuatorManagerInterface
{
 public:
  virtual ~ActuatorManagerInterface() {}

  virtual bool HasAbsoluteEncoder() = 0;

  virtual bool SetOutputPosition(double position) = 0;

  virtual double GetActualPosition() = 0;
};
}  // namespace fastcat

#endif