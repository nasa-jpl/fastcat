# Example Fastcat Configurations

This directory contains a short list of working, tested Fastcat configuration files.

## Offline Configs

### faulter_config.yaml

Many Fastcat devices have the ability to emit faults, however a generic capability was needed to keep testbed configuration flexible. The Faulter device can be used to add custom fault handling to the bus. Say, you have a load cell that is protecting sensitive hardware you could create Conditional device that trips for high loads and pass the Conditional device Fastcat signal to a Faulter device that would fault all other devices on the Fastcat bus, protecting your hardware. This example configuration uses a Conditional and Faulter to create a simple fault monitor system using the Faulter device.

### fts_device_config.yaml

This configuration uses a signal generator to create a FTS analog input, which is in turn transformed to another sensing frame by means of a Virtual FTS device. 

### offline_jsd_config.yaml

This configuration tests the offline versions of the JSD EtherCAT devices.

### robotic_arm_config.yaml

A configuration with seven actuators as a simulator for a robotic arm.

### signal_processing_config.yaml

Captures common signal processing step to acquire a raw signal, filter, and apply a linear function to the output to get a processed signal that can then be used internal to Fastcat by another device or external to Fastcat

### zero_latency_config.yaml

Highlights the internal reordering performed by Fastcat. The device list here is ordered such that a naive processing order would result in several loops of latency. During initialization, Fastcat correctly reorders these signals such that there is no latency between the devices that use Fastcat signals.

## Online Configs

### Testing Setup

A test setup called a Bluebox "BBx" is contains power electronics, a Beckhoff stack, and 6 Elmo Gold Drives. This common hardware is accessible to developers within our organization who wish to test new drivers or features before field deployments.

Contact Alex Brinkman (alexander.brinkman@jpl.nasa.gov) if you have questions about this hardware configuration.

### bbx_heater_config.yaml

This configuration uses a electrical RC low pass filter to emulate a heater-temperature system and implements a bang-bang controller to keep the 'temperature' within an acceptable range. An EGD is used as a visual display of the 'temperature'. 

### bbx_motor_sine_config.yaml

This configuration uses a signal generator to command several EGD devices to follow a sine wave position profile.

## Device Coverage

 Fastcat Device Coverage:

| Config File                           | Actuator | Commander | Conditional | Faulter | Filter | FTS  | Function | Schmitt Trigger | Signal Generator | Virtual FTS |
| ------------------------------------- | -------- | --------- | ----------- | ------- | ------ | ---- | -------- | --------------- | ---------------- | ----------- |
| offline/faulter_config.yaml           |          |           | X           | X       |        |      |          |                 | X                |             |
| offline/fts_device_config.yaml        |          |           |             |         |        | X    |          |                 | X                | X           |
| offline/offline_jsd_config.yaml       |          | X         |             |         |        |      | X        |                 | X                |             |
| offline/robotic_arm_config.yaml       | X        |           |             |         |        |      |          |                 |                  |             |
| offline/signal_processing_config.yaml |          |           | X           |         | X      |      | X        | X               | X                |             |
| offline/zero_latency_config.yaml      |          | X         |             |         |        |      | X        | X               | X                |             |
| online/bbx_heater_config.yaml         |          | X         |             |         | X      |      | X        | X               |                  |             |
| online/bbx_motor_sine_config.yaml     |          |           |             |         |        |      |          |                 |                  |             |

JSD Device Coverage:

| Config File                           | EGD  | EL2124 | EL3208 | EL3602 | JED  |
| ------------------------------------- | ---- | ------ | ------ | ------ | ---- |
| offline/faulter_config.yaml           |      |        |        |        |      |
| offline/fts_device_config.yaml        |      |        |        |        |      |
| offline/offline_jsd_config.yaml       | X    | X      | X      | X      |      |
| offline/robotic_arm_config.yaml       |      |        |        |        |      |
| offline/signal_processing_config.yaml |      |        |        |        |      |
| offline/zero_latency_config.yaml      |      | X      |        |        |      |
| online/bbx_heater_config.yaml         | X    | X      |        | X      |      |
| online/bbx_motor_sine_config.yaml     | X    |        |        |        |      |