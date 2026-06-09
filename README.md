# fastcat

C++ EtherCAT Device Command & Control Library

### Documentation

To learn more about fastcat, checkout the following documents:

- [Fastcat Primer](doc/fastcat_primer.md)
- [Complete list of Fastcat Device Configuration YAML Parameters](doc/fastcat_device_config_parameters.md)
- [Command-line Utilities](doc/utilities.md) — `jsd_slaveinfo`, `elmo_vel_profile`, `elmo_pos_profile`
- 2021 Aeroconf paper submission `Fastcat: An Open-Source Library for Composable
  EtherCAT Control Systems`
- README for build details
- The API documentation

### Prerequisites

Fastcat has been tested on Ubuntu 20.04, 22.04 and 24.04, though it should work on older versions of Ubuntu with minor revisions to these steps.

```bash
$ sudo apt install libyaml-cpp-dev libreadline-dev doxygen graphviz
```

The `fcviz` utility is configured as a `uv` script, so its Python dependencies do not need to be installed globally:

Install `uv` using Astral's official installer:

```bash
$ curl -LsSf https://astral.sh/uv/install.sh | sh
```

Installation options and other platforms are documented by Astral at <https://docs.astral.sh/uv/getting-started/installation/>.

```bash
$ uv run fcviz/fcviz.py example_configs/paper_examples/paper_faulter_config.yaml
```

If you do not want to use `uv`, check the inline dependency metadata at the top of `fcviz/fcviz.py` and install those packages in your preferred Python environment before running the script directly.

If you need to regenerate code with `fcgen`, install Ubuntu's `python3-cogapp` package. On some systems, you may need to enable the `universe` repository first:

```bash
$ sudo add-apt-repository universe
$ sudo apt update
$ sudo apt install python3-cogapp
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

### Network Port Access for EtherCAT

The shipped utilities (`jsd_slaveinfo`, `elmo_vel_profile`, `elmo_pos_profile`) and any modules built using fastcat open raw EtherCAT sockets and need `CAP_NET_ADMIN` + `CAP_NET_RAW` permissions.
This can be achieved either by running the binary as the root user, or by using the `sudo setcap` command to permit non-root users to access the raw socket.
For example, you can permit a non-root user to run the `elmo_vel_profile` binary with the command:

```bash
$ sudo setcap cap_net_admin,cap_net_raw=eip /absolute/path/to/build/bin/elmo_vel_profile
```

**The raw socketr permissions are cleared on every relink**, so you must re-run `sudo setcap` after each rebuild.

#### Auto-setcap during build

For dev machines, build with `-DAUTO_SETCAP=ON` to automatically run `sudo setcap` on `elmo_vel_profile` and `elmo_pos_profile` after each link:

```bash
$ cmake -S . -B build -DAUTO_SETCAP=ON
$ cmake --build build -j
```

The option defaults to **OFF** so fastcat builds cleanly on machines without sudoers configured (CI, headless servers, etc.). `jsd_slaveinfo` is built by the JSD dependency and is not covered by this flag — set its capabilities manually.

#### Password-less sudo for `setcap`

`AUTO_SETCAP=ON` only works without prompting if `sudo setcap` is whitelisted in sudoers. Run `sudo visudo` and add a single line (replace `<your_username>` with your account name):

```
<your_username> ALL=(ALL) NOPASSWD: /usr/sbin/setcap
```

This narrowly grants password-less access to `setcap` only — `sudo` for any other command still prompts as usual.

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
$ cd build
$ make doc
```

The output documentation is created in the directory `doxygen_html` and can be opened by any web browser from the root `index.html` webpage.

The repository also includes a GitHub Actions workflow that builds this `doc` target and publishes it to GitHub Pages on pushes to `master`. Once GitHub Pages is enabled for the repository with GitHub Actions as the source, the published site will be available at <https://nasa-jpl.github.io/fastcat/>.

### Using fastcat in your Project

We recommend using the CMake `FetchContent` utility to acquire fastcat and its upstream dependencies.

```cmake
include(FetchContent)
FetchContent_Declare(fastcat
    GIT_REPOSITORY git@github.com:nasa-jpl/fastcat.git
    GIT_TAG v0.13.13
    )
FetchContent_MakeAvailable(fastcat)
```

It is always recommend you specify your dependency to a tagged release (e.g. `GIT_TAG v0.13.13`) so updates to master cannot break your build (NOT `GIT_TAG master`).

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

## License

fastcat is licensed under the Apache License 2.0. See [LICENSE](LICENSE) for more details.
