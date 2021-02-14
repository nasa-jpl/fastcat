**commander**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| device_cmd_name | name of device controlled by commander |
| device_cmd_type | type of device controlled by commander |
| observed_device_name | name of device from which signal is read |
| request_signal_name | signal to be read from specified device |

**signal_generator (sine wave)**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| signal_generator_type | type of signal to be generated |
| angular_frequency |  |
| phase |  |
| amplitude |  |
| offset |  |

**signal_generator (sawtooth)**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| signal_generator_type | type of signal to be generated |
| slope |  |
| max |  |
| min |  |
| range |  |
| modulo |  |

**function**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| function_type | type of function to be applied |
| order | order of function |
| coefficients | coefficients of function |
| observed_device_name | name of device from which signal is read |
| request_signal_name | signal to be read from specified device |

**conditional**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| conditional_type |  |
| compare_rhs_value |  |
| observed_device_name | name of device from which signal is read |
| request_signal_name | signal to be read from specified device |

**schmitt_trigger**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| low_threshold |  |
| high_threshold |  |
| observed_device_name | name of device from which signal is read |
| request_signal_name | signal to be read from specified device |

**filter**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| filter_type | type of filter to be applied to signal |
| buffer_size |  |
| observed_device_name | name of device from which signal is read |
| request_signal_name | signal to be read from specified device |

**egd**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| motor_rated_current_amps |  |
| max_current_amps |  |
| max_motor_speed |  |

**el3208**

| Parameter | Description |
| --------- | ----------- |
| name | device name |
| element | type of hardware element connected to each channel |
| connection | number of wires in each channel's connection |
| wire resistance | resistance of wires in Ohms |

**el3602**

| Parameter | Description |
| --------- | ----------- |
| name | device name |

**el2124** 

| Parameter | Description |
| --------- | ----------- |
| name | device name |
