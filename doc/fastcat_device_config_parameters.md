# Fastcat Device List

**JSD and Offline Devices**

For every `JSD Device` there is an `Offline Device` to emulate the behavior of the hardware.

| Name     | Manufacturer | Description                         |
| -------- | ------------ | ----------------------------------- |
| Actuator | Elmo         | EGD with extra features             |
| Egd      | Elmo         | Elmo Gold Drive line of controllers |
| El3208   | Beckhoff     | 8-channel RTD Input                 |
| El3602   | Beckhoff     | 2-channel +/-10v Diff. Analog Input |
| El2124   | Beckhoff     | 4-channel 5v Digital Output         |

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

| Parameter Name            | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| `bus/type`                | Specifies which devices are on this bus {`offline_bus`, `jsd_bus`, `fastcat_bus`} |
| `bus/ifname`              | The interface name only used by JSD bus to indicate which NIC is used for the EtherCAT Master |
| `bus/enable_autorecovery` | `jsd_bus`only. Enables a feature that may attempt to recover the bus if the working counter changes |

#### bus/type

Only 3 special strings are permitted to define the bus types. All devices within a bus are handled by the same context manager so you cannot specify `JSD Devices` on the same bus as `Fastcat Devices` for example.

Each `jsd_bus` denotes a unique EtherCAT Master.

Multiple buses of any time can be supported.

#### bus/ifname

Functionally only used by the `jsd_bus` to specific which Network Interface Controller (NIC) is being used for that EtherCAT Master instance. 

Tip: use `ip a` or `ifconfig` to check your list of interfaces on Debian/Ubuntu

#### bus/enable_autorecovery

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

# Device Parameters

All devices configurations are contained in a `devices` YAML sequence within a bus. 

Every device has a `device_class` that matches the fastcat API class name. A special device class called `IGNORE` is used on `jsd_bus` to tell the EtherCAT master to ignore certain slaves when initializing slaves and exchanging PDOs. The following snippet shows how to use the `IGNORE` device class to ignore the EK1100 passive EtherCAT Bus Coupler.

``` yaml 
buses:
  - type: jsd_bus
    ... 
    devices:
    - device_class: IGNORE # EK1100 Coupler
    - device_class: ... 
```

Every device has a `name` parameter that must be unique for all devices on the bus. The `name` parameter is important because this is how commands are marshaled from the Manager command queue to each unique device.

These two parameters, `device_class` and `name`, are not explicitly covered in the following device configuration parameter descriptions but must be specified for each and every device. 

## JSD and Offline Device Parameters

## Actuator

Engineering Units (EU) are radians for revolute actuators and meters for linear actuators.

| Parameter                       | Description                                                  |
| ------------------------------- | ------------------------------------------------------------ |
| `actuator_type`                 | Either `revolute` or `linear`. This dictates the EU          |
| `gear_ratio`                    | The gear ratio relating motor speed to actuator output (e.g. input/output) |
| `counts_per_rev`                | The number of sensor counts per motor revolution             |
| `max_speed_eu_per_sec`          | Maximum actuator Output speed this drive may be commanded    |
| `max_accel_eu_per_sec2`         | Max actuator output accel this drive may be commanded        |
| `over_speed_multiplier`         | Multiplicative factor over `max_speed_eu_per_sec` that triggers a fault |
| `vel_tracking_error_eu_per_sec` | Fault if tracking error `fabs(Actual Vel - Cmd Vel)` exceeds this parameter |
| `pos_tracking_error_eu_per_sec` | Fault if tracking error `fabs(Actual Pos - Cmd Pos)` exceeds this parameter |
| `peak_current_limit_amps`       | Peak instantaneous current permitted to actuator             |
| `peak_current_time_sec`         | Max apply duration of Peak current before dropping down to Max Continuous current |
| `continuous_current_limit_amps` | Max current permitted to actuator                            |
| `torque_slope_amps_per_sec`     | Rate to apply torque in certain profiled torque command modes |
| `low_pos_cal_limit_eu`          | Lower Position Limit typically corresponding to a hardstop. Used for Calibration Command |
| `low_pos_cmd_limit_eu`          | Lowest allowable command position value                      |
| `high_pos_cal_limit_eu`         | Upper Position Limit typically corresponding to a hardstop. Used for Calibration Command |
| `high_pos_cmd_limit_eu`         | Highest allowable command position value                     |
| `holding_duration_sec`          | Duration to hold position after reset or after a motion command before re-engaging brakes |
| `egd_brake_engage_msec`         | How long it takes to re-engage the brakes                    |
| `egd_brake_disengage_msec`      | How long it takes to disengaged the brakes                   |
| `egd_crc`                       | CRC of the flashed Elmo parameter set                        |
| `egd_drive_max_current_limit`   | The Maximum drive current for the Elmo Gold Drive            |
| `smooth_factor`                 | Affects controller smoothing, defaults to `0`                |

### Implementation Notes

* The Actuator device Class is based of the JSD Elmo Gold Drive (Egd) device
* Wherever possible, the responsibility to check faults is delegated down to the EtherCAT Slave (rather than keep that logic at the Application layer) to promote the fastest fault-checking possible
  * Position and Velocity Tracking faults are delegated to the Egd slave
  * Overspeed faults are delegated to the Egd slave
* The Egd must be tuned prior to use within Fastcat. 
  * Some parameters like `max_speed_eu_per_sec` are checked against these internal parameters and cannot be exceeded with flashing the drive. 
  * Modifying any of the GPRM parameters and calling `SV` will change the `egd_crc`
  * The Elmo CRC value is checked to make sure the YAML parameters align with the drive parameters

### Example

``` yaml
    - device_class:                  Actuator
      name:                          tool
      actuator_type:                 revolute # eu = radians
      gear_ratio:                    19
      counts_per_rev:                6
      max_speed_eu_per_sec:          100
      max_accel_eu_per_sec2:         10
      over_speed_multiplier:         3.0
      vel_tracking_error_eu_per_sec: 0.157
      pos_tracking_error_eu:         0.157
      peak_current_limit_amps:       25.46
      peak_current_time_sec:         1.0
      continuous_current_limit_amps: 7.5
      torque_slope_amps_per_sec:     2.0
      low_pos_cal_limit_eu:          -1e15
      low_pos_cmd_limit_eu:          -1e15
      high_pos_cmd_limit_eu:         1e15
      high_pos_cal_limit_eu:         1e15
      holding_duration_sec:          5.0
      egd_brake_engage_msec:         10 
      egd_brake_disengage_msec:      10
      egd_crc:                       -3260
      egd_drive_max_current_limit:   10
      smooth_factor:                 0
```



## Egd (Elmo Gold Drive) TODO



## El3208 (8-channel RTD Input)

| Parameter         | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| `element`         | type of hardware element connected to each channel           |
| `connection`      | number of wires in each channel's connection {`2WIRE`, `3WIRE`, `4WIRE`, `NOT_CONNECTED`} |
| `wire_resistance` | resistance of wires in Ohms, used to improve the temperature estimate if line resistance is known |
| `low_threshold`   | Fault issued if temperature drops below this threshold (deg C.) |
| `high_threshold`  | Fault issued if temperature exceeds this threshold (deg C.)  |

Allowable `element` values (See the EL3208 Beckhoff Manual `0x80n0:19` Data Object)

* `PT100`
* `NI100`
* `PT1000`
* `PT500`
* `PT200`
* `NI1000`
* `NI1000_TK1500`
* `NI120`
* `OHMS4096` - Outputs resistance instead of temp. Senses up to 4096 Ohms (1/16 Ohm resolution)
* `OHMS1024` - Outputs resistance instead of temp. Senses up to 1024 Ohms (1/64 Ohm resolution)
* `KT100_ET_AL`
* `NOT_CONNECTED`

#### Example

```yaml
- device_class: El3208
  name: el3208_1
  element:         [PT100, NI100, PT100, PT100, PT100, PT100, OHMS1024, NOT_CONNECTED]
  connection:      [2WIRE, 2WIRE, 3WIRE, 2WIRE, 4WIRE, 2WIRE, 2WIRE,    NOT_CONNECTED]
  wire_resistance: [0,     0,     0,     0,     0,     0,     0,        0            ]
  low_threshold:   [-100,  -100,  -100,  -100,  -1,     -100,  -100,     1           ]
  high_threshold:  [100,   100,   100,   100,   1,    100,   100,      -1            ]
```


## El3602 (2-channel +/-10v Diff. Analog Input)

| Parameter   | Description                 |
| ----------- | --------------------------- |
| `range_ch1` | Voltage Range for Channel 1 |
| `range_ch2` | Voltage Range for Channel 2 |

The permitted range values are:

* `10V` - +/- 10 volts
* `5V` - +/- 5 volts
* `2_5V` - Corresponding to +/- 2.5 volts
* `75MV` - Corresponding to +/- 75 **milliVolts**
* `200MV` - Corresponding to +/- 200 **millivolts**

#### Example

``` yaml
- device_class: El3602
  name: el3602_1
  range_ch1: 10V
  range_ch2: 5V
```

## El2124 (4-channel 5v Digital Output)

**The El2124 device has no configuration parameters**

#### Example

``` yaml
- device_class: El2124
  name: el2124_1
```



## Fastcat Device Parameters

**@TODO**



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

