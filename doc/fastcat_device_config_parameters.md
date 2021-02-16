# Fastcat Device List

**JSD and Offline Devices**

For every `JSD Device` there is an `Offline Device` to emulate the behavior of the hardware.

| Name   | Manufacturer | Description                              |
| ------ | ------------ | ---------------------------------------- |
| El2124 | Beckhoff     | 4-channel 5v Digital Output              |
| El3208 | Beckhoff     | 8-channel RTD Input                      |
| El3602 | Beckhoff     | 2-channel +/-10v Diff. Analog Input      |
| Egd    | Elmo         | Elmo Gold Drive - DS402 motor controller |

**Fastcat Devices**

`Fastcat Devices` do not have any physical EtherCAT hardware equivalent. The operate identically regardless if they consume  `Offline Device` or `JSD Device`  `Signals`

| Name            | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| Commander       | Routes `Signals` to command arguments and issues internal commands to other devices |
| Conditional     | Logical test for signal with a boolean state data field      |
| Faulter         | Emits a fault if a the input signal != 0                     |
| Filter          | Supports Digital AB and Moving Average filtering on Signals  |
| Fts             | Reads in 6 'raw' signals and multiplies them through a 6x6 calibration matrix to compute a wrench |
| Function        | Applies a function to a single input signal (e.g. parametrized N-order polynomial) |
| Pid             | Applies a PID Controller to signal with deadband and persistence arguments |
| Saturation      | Applies upper and lower saturation limits to a signal        |
| SchmittTrigger  | Simple software debounce trigger, parameterized with upper and lower thresholds |
| SignalGenerator | Generates a parameterized signal (e.g. sine wave or sawtooth) useful for testing devices and configurations |
| VirtualFts      | Reads in 6 signals (corresponding to a wrench) and applies the adjoint wrench transformation to different 6DOF pose |

---

## YAML Parameters

## Fastcat Global Parameters

| Parameter Name                       | Description                                                  | Type   | Recommended Value |
| ------------------------------------ | ------------------------------------------------------------ | ------ | ----------------- |
| `target_loop_rate_hz`                | Loop Rate of Fastcat Application                             | double | 100 - 500         |
| `zero_latency_required`              | Controls Manager reaction if circular `Signal` dependencies exist | bool   | True              |
| `actuator_position_directory`        | Parent directory of actuator saved position file             | string | /tmp/             |
| `actuator_fault_on_missing_pos_file` | If true, Fastcat will fail during initialization if a saved pos file does not exist | bool   | True              |

#### target_loop_rate_hz

Some devices use of the loop period to provide better performance so the application should try to honor this loop rate. For example, the EGD slave uses the loop period to perform some interpolation between commands. The `fastcat::manager` class has a getter for this loop rate parameter, `double GetTargetLoopRate()` so applications can easily extract this parameter from the library without redundant parameterization or YAML parsing.

#### zero_latency_required

During initialization, devices will be reordered by the manager to ensure that the processing order ensures that `Signal` consumers follow `Signal` producers. If there exists a cyclic dependency (e.g. A observes B and B observes A) this parameter controls how the manager reacts when 'zero latency' cannot be achieved within the fastcat device bus. If True, the manager will fault during configuration. If False, the manager will print out a helpful warning message and ignore the cyclic dependency.

#### actuator_position_directory

To work seamlessly with actuators, the manager may need to cache the last known position of actuators if they are using any non-absolute position sensor (e.g. incremental encoder, Hall-effect sensor) The Manager will look inside the `actuator_position_directory` for a pre-existing `fastcat_saved_positions.yaml` file to restore position from this file. 

#### actuator_fault_on_missing_pos_file

The `fastcat_saved_positions.yaml` may not exist for any number of reasons. If this file does not exist, this parameter controls how the manager reacts. if True, then the manager will fault and not initialize. if False, the assumed startup position for all actuators is `0` and when a new `fastcat_saved_positions.yaml` fill will be created when the manager is shutdown by the application.

#### Examples

``` YAML
# Recommended For Online Hardware
fastcat:
	target_loop_rate_hz:                500   # 100 - 500
	zero_latency_required:              True  # Always
	actuator_position_directory:        /cal/ # or any other global location on your filesystem
	actuator_fault_on_missing_pos_file: True  # Online - True, Offline - False
```

``` yaml
# Recommended For Offline Hardware
fastcat:
	target_loop_rate_hz:                500   # 100 - 500
	zero_latency_required:              True  # Always
	actuator_position_directory:        /tmp/ # Recommended this is different from the Online path
	actuator_fault_on_missing_pos_file: False # Let Fastcat Create this for us!
```

---

## Bus Parameters

Fastcat can support multiple buses of each type

| Parameter Name              | Description                                                  | Type   | Permitted Value                           |
| --------------------------- | ------------------------------------------------------------ | ------ | ----------------------------------------- |
| `buses/type`                | Specifies which devices are on this bus                      | string | {`offline_bus`, `jsd_bus`, `fastcat_bus`} |
| `buses/ifname`              | The interface name only used by JSD bus to indicate which NIC is used for the EtherCAT Master | string |                                           |
| `buses/enable_autorecovery` | `jsd_bus`only. Enables a feature that may attempt to recover the bus if the working counter changes | Bool   |                                           |

#### type

Only 3 special strings are permitted to define the bus types. All devices within a bus are handled by the same context manager so you cannot specify `JSD Devices` on the same bus as `Fastcat Devices` for example.

Each `jsd_bus` denotes a unique EtherCAT Master.

Multiple buses of any time can be supported.

#### ifname

Functionally only used by the `jsd_bus` to specific which Network Interface Controller (NIC) is being used for that EtherCAT Master instance. 

Tip: use `ip a` or `ifconfig` to check your list of interfaces on Debian/Ubuntu

#### enable_autorecovery

This feature aims to recover the EtherCAT Master if a change in working counter (WKC) is detected. This can occur if a slave is not responding properly, the physical bus topology has been changed, or some intermittent power/communication issue is present. 

This feature is NOT real-time safe so it is not recommended to be used beyond debugging efforts. 

#### Examples

``` yaml
buses:
	- type: offline_bus
	  ifname: offline_1
	  devices:
	    ... # Only offline JSD devices can be specified here
	    
	- type: jsd_bus
	  ifname: eno1
	  enable_autorecovery: False
	  devices:
	    ... # Only online JSD devices can be specified here
	    
	- type: fastcat_bus
	  ifname: fastcat_1
	  devices:
	    ... # Only fastcat devices can be specified here
	    
	... # add more {offline_bus, jsd_bus, fastcat_bus} buses as desired
```

---

## JSD and Offline Device Parameters

**egd**

| Parameter                | Description |
| ------------------------ | ----------- |
| name                     | device name |
| motor_rated_current_amps |             |
| max_current_amps         |             |
| max_motor_speed          |             |

**el3208**

| Parameter       | Description                                        |
| --------------- | -------------------------------------------------- |
| name            | device name                                        |
| element         | type of hardware element connected to each channel |
| connection      | number of wires in each channel's connection       |
| wire resistance | resistance of wires in Ohms                        |

**el3602**

| Parameter | Description |
| --------- | ----------- |
| name      | device name |

**el2124** egd**

| Parameter                | Description |
| ------------------------ | ----------- |
| name                     | device name |
| motor_rated_current_amps |             |
| max_current_amps         |             |
| max_motor_speed          |             |

**el3208**

| Parameter       | Description                                        |
| --------------- | -------------------------------------------------- |
| name            | device name                                        |
| element         | type of hardware element connected to each channel |
| connection      | number of wires in each channel's connection       |
| wire resistance | resistance of wires in Ohms                        |

**el3602**

| Parameter | Description |
| --------- | ----------- |
| name      | device name |

**el2124** 

## Fastcat Device Parameters

**commander**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| device_cmd_name      | name of device controlled by commander   |
| device_cmd_type      | type of device controlled by commander   |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**signal_generator (sine wave)**

| Parameter             | Description                    |
| --------------------- | ------------------------------ |
| name                  | device name                    |
| signal_generator_type | type of signal to be generated |
| angular_frequency     |                                |
| phase                 |                                |
| amplitude             |                                |
| offset                |                                |

**signal_generator (sawtooth)**

| Parameter             | Description                    |
| --------------------- | ------------------------------ |
| name                  | device name                    |
| signal_generator_type | type of signal to be generated |
| slope                 |                                |
| max                   |                                |
| min                   |                                |
| range                 |                                |
| modulo                |                                |

**function**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| function_type        | type of function to be applied           |
| order                | order of function                        |
| coefficients         | coefficients of function                 |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**conditional**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| conditional_type     |                                          |
| compare_rhs_value    |                                          |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**schmitt_trigger**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| low_threshold        |                                          |
| high_threshold       |                                          |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**filter**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| filter_type          | type of filter to be applied to signal   |
| buffer_size          |                                          |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

****commander**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| device_cmd_name      | name of device controlled by commander   |
| device_cmd_type      | type of device controlled by commander   |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**signal_generator (sine wave)**

| Parameter             | Description                    |
| --------------------- | ------------------------------ |
| name                  | device name                    |
| signal_generator_type | type of signal to be generated |
| angular_frequency     |                                |
| phase                 |                                |
| amplitude             |                                |
| offset                |                                |

**signal_generator (sawtooth)**

| Parameter             | Description                    |
| --------------------- | ------------------------------ |
| name                  | device name                    |
| signal_generator_type | type of signal to be generated |
| slope                 |                                |
| max                   |                                |
| min                   |                                |
| range                 |                                |
| modulo                |                                |

**function**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| function_type        | type of function to be applied           |
| order                | order of function                        |
| coefficients         | coefficients of function                 |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**conditional**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| conditional_type     |                                          |
| compare_rhs_value    |                                          |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**schmitt_trigger**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| low_threshold        |                                          |
| high_threshold       |                                          |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

**filter**

| Parameter            | Description                              |
| -------------------- | ---------------------------------------- |
| name                 | device name                              |
| filter_type          | type of filter to be applied to signal   |
| buffer_size          |                                          |
| observed_device_name | name of device from which signal is read |
| request_signal_name  | signal to be read from specified device  |

