# fastcat

C++ EtherCAT Device Command & Control Library

### Documentation

To learn more about fastcat, checkout the following documents:

- [Fastcat Primer](doc/fastcat_primer.md)
- [Complete list of Fastcat Device Configuration YAML Parameters](doc/fastcat_device_config_parameters.md)
- 2021 Aeroconf paper submission `Fastcat: An Open-Source Library for Composable
  EtherCAT Control Systems`
- README for build details
- The API documentation (build locally using doxygen - Github hosting still a work-in-progress)

### Prerequisites

Fastcat has ben tested on Ubuntu 20.04, though it should work on older versions of Ubuntu with minor revisions to these steps. 

```bash
$ sudo apt install libyaml-cpp-dev libreadline-dev doxygen python3-pip
$ sudo pip3 install pyaml cogapp graphviz ipython==7.9
```

### Building

To build fastcat from source:
```bash
$ git clone git@github.com:nasa-jpl/fastcat.git
$ cd fastcat
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### Tests

The following commands will execute the unit tests:

```bash
$ cd build
$ cmake ..
$ make
$ make test
$ make memcheck  # note valgrind is required to perform memory checking
```

### API Documentation

```bash
# Install dependencies for Ubuntu
$ sudo apt install doxygen graphviz

# use the build system to generate the code for you!
$ cd build
$ make doc
```

The output documentation is created in the directory `doxygen_html` and can be opened by any web browser from the root `index.html` webpage.

### Using fastcat in your Project

We recommend using the CMake `FetchContent` utility to acquire fastcat and its upstream dependencies.

```cmake
include(FetchContent)
FetchContent_Declare(fastcat
    GIT_REPOSITORY git@github.com:nasa-jpl/fastcat.git
    GIT_TAG v0.4.3
    )
FetchContent_MakeAvailable(fastcat)
```

It is always recommend you specify your dependency to a tagged release (`GIT_TAG v0.4.3`) so updates to master cannot break your build (NOT `GIT_TAG master`).

### Manager API Usage

Fastcat provides two initialization patterns to accommodate different application architectures:

#### Standard Initialization (Single-Phase)

For simple applications, use `ConfigFromYaml()` which handles both configuration parsing and hardware initialization:

```cpp
#include "fastcat/manager.h"

fastcat::Manager manager;
YAML::Node node = YAML::LoadFile("config.yaml");

if (!manager.ConfigFromYaml(node)) {
    // Handle error
    return false;
}

// Manager is ready - start your control loop
while (running) {
    manager.Process();
}
```

#### Split Initialization (Two-Phase)

For applications that need to perform time-consuming operations between configuration parsing and hardware initialization (e.g., ROS node setup), use the split API to prevent EtherCAT watchdog timeouts:

```cpp
#include "fastcat/manager.h"

fastcat::Manager manager;
YAML::Node node = YAML::LoadFile("config.yaml");

// Phase 1: Parse YAML and create device objects (no hardware init)
if (!manager.CreateConfigFromYaml(node)) {
    // Handle error
    return false;
}

// Perform time-consuming setup operations here
// Example: Create ROS publishers, subscribers, services, etc.
// This won't cause EtherCAT slaves to timeout because they're not yet in OP state

// Phase 2: Initialize EtherCAT hardware and transition slaves to OP state
if (!manager.InitHardware()) {
    // Handle error
    return false;
}

// IMPORTANT: Start your control loop immediately after InitHardware()
// to prevent SM watchdog timeouts (~100ms)
while (running) {
    manager.Process();
}
```

**When to use split initialization:**
- Your application performs time-consuming setup (>50ms) after configuration
- You're integrating with frameworks that have initialization overhead (ROS, middleware, etc.)
- You need to query device information before hardware initialization

**Note:** The `Process()` loop must maintain the target loop rate specified in your YAML configuration to prevent EtherCAT watchdog timeouts. 

### Semantic Versioning

fastcat uses Semantic versioning to help applications reason about the software as updates are continuously rolled out. Tailored to fastcat, the Semver rules are as follows:

* Major Versions will denote changes to the API or YAML Configuration parameters, which may break some user applications.
* Minor Versions will denote new features or driver additions that do not break user applications.
* Patch Versions will denote bug fixes or minor improvements and will not break user applications.

Violations of these rules will be considered errors and should be patched immediately. Please open an issue if you find a violation.

**Note**

Major version `0` indicates the API is still considered experimental and subject to change with any new release. These rules will be strictly followed after a Major version `1` release. 

