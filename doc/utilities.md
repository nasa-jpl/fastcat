# fastcat Utilities

fastcat ships three command-line utilities. After a top-level build, all of
them land in `build/bin/`:

| Binary | Purpose |
|---|---|
| `jsd_slaveinfo` | Enumerate EtherCAT slaves on a given interface |
| `elmo_vel_profile` | Run a trapezoidal velocity profile on a single actuator |
| `elmo_pos_profile` | Run a trapezoidal position profile on a single actuator |

`jsd_slaveinfo` is built as part of fastcat's [JSD](https://github.com/nasa-jpl/jsd)
dependency. The two `elmo_*_profile` binaries are native fastcat utilities
under `utils/`.

## Capabilities

All three utilities open raw EtherCAT sockets, which requires `CAP_NET_ADMIN`
and `CAP_NET_RAW`. Grant the capabilities directly to the binary instead of
running under `sudo`:

```
sudo setcap cap_net_admin,cap_net_raw=eip build/bin/<binary>
```

`setcap` requires the absolute path. **Capabilities are cleared on every
rebuild**, so re-run `setcap` after each `cmake --build`.

### Auto-setcap during build (optional)

For dev machines with password-less sudo configured, build with
`-DAUTO_SETCAP=ON` to automatically run `setcap` on the profile binaries
after each link:

```
cmake -S . -B build -DAUTO_SETCAP=ON
cmake --build build -j
```

The option defaults to **OFF** so that fastcat builds cleanly on machines
without sudoers configured (CI, headless servers, etc.).

To enable password-less `setcap`, run `sudo visudo` and add:

```
<your_username> ALL=(ALL) NOPASSWD: /usr/sbin/setcap
```

Note: `AUTO_SETCAP` only applies to `elmo_vel_profile` and `elmo_pos_profile`.
`jsd_slaveinfo` is built by the JSD dependency and is not covered by this
option — set its capabilities manually after each rebuild.

---

## `jsd_slaveinfo`

Enumerates EtherCAT slaves on the given network interface and prints a
summary of each one (vendor ID, product code, configured PDOs, etc.). Useful
for confirming the bus is wired correctly and that every expected drive shows
up before launching a fastcat-based application.

### Usage

```
./build/bin/jsd_slaveinfo <ifname>
```

Example:

```
./build/bin/jsd_slaveinfo eth_ecat
```

For more detailed documentation see the JSD project's own
[README](https://github.com/nasa-jpl/jsd).

---

## `elmo_vel_profile`

Runs a trapezoidal velocity profile on one actuator described by a fastcat
YAML config. The profile accelerates from rest to a cruise speed, holds for a
specified duration, then decelerates back to zero. Telemetry is streamed to a
timestamped CSV file in the current working directory.

### Usage

```
./build/bin/elmo_vel_profile <config> <actuator_name> <accel> <cruise_speed> <cruise_duration>
```

| Argument | Description |
|---|---|
| `config` | Path to fastcat YAML config (relative to CWD) |
| `actuator_name` | Must match a `name:` field of a device in the YAML |
| `accel` | Trapezoid acceleration / deceleration (rad/s²) |
| `cruise_speed` | Cruise velocity (rad/s) |
| `cruise_duration` | Hold time at cruise speed (s) |

### Example

```
cd build/bin
./elmo_vel_profile ../../example_configs/single_elmo.yaml gold_act_1 2.0 5.0 3.0
```

### Telemetry output

Written to `<YYYYMMDD>_<HHMMSS>_vel_prof_telem.csv` in CWD with one row per
control tick:

| Column | Units | Description |
|---|---|---|
| `unix_time_s` | seconds since 1970-01-01 UTC | Wall-clock timestamp for correlating with other telemetry sources |
| `relative_time_s` | seconds | Time since the start of the control loop |
| `cmd_velocity_rad_s` | rad/s | Velocity commanded by the trapezoid generator |
| `actual_velocity_rad_s` | rad/s | Velocity reported by the drive (heavily quantized at low encoder resolution) |
| `actual_current_A` | A | Motor current reported by the drive |

---

## `elmo_pos_profile`

Runs a trapezoidal position profile on one actuator. The trapezoid shape is
chosen automatically based on the requested distance: triangular if the move
is short enough that the drive can't reach `max_velocity` before having to
decelerate, otherwise trapezoidal with a cruise phase at `max_velocity`.
Negative `relative_position` values move in the negative direction.

### Usage

```
./build/bin/elmo_pos_profile <config> <actuator_name> <accel> <max_velocity> <relative_position>
```

| Argument | Description |
|---|---|
| `config` | Path to fastcat YAML config (relative to CWD) |
| `actuator_name` | Must match a `name:` field of a device in the YAML |
| `accel` | Acceleration / deceleration (rad/s²) |
| `max_velocity` | Cruise velocity cap (rad/s) |
| `relative_position` | Position change relative to the current position (rad); may be negative |

### Example

```
cd build/bin
./elmo_pos_profile ../../example_configs/single_elmo.yaml gold_act_1 2.0 5.0 3.0
./elmo_pos_profile ../../example_configs/single_elmo.yaml gold_act_1 2.0 5.0 -3.0
```

### Telemetry output

Written to `<YYYYMMDD>_<HHMMSS>_pos_prof_telem.csv` in CWD with one row per
control tick:

| Column | Units | Description |
|---|---|---|
| `unix_time_s` | seconds since 1970-01-01 UTC | Wall-clock timestamp |
| `relative_time_s` | seconds | Time since the start of the control loop |
| `cmd_position_rad` | rad | Position commanded by the trapezoid generator |
| `actual_position_rad` | rad | Position reported by the drive |
| `actual_velocity_rad_s` | rad/s | Velocity reported by the drive (noisy at low resolution) |
| `actual_current_A` | A | Motor current reported by the drive |

---

## Loop rate guidance

Both profile utilities take their loop rate from the YAML's
`target_loop_rate_hz`. **64 Hz is the recommended minimum.** Each binary
prints a warning at startup if the configured rate is below 64 Hz, but does
not refuse to run.

The reason is encoder quantization noise on the velocity feedback signal.
For a typical low-resolution encoder (42 counts per revolution), the
single-tick velocity resolution is roughly `2π × loop_rate / counts_per_rev`:

| Loop rate | Velocity resolution per encoder count |
|---|---|
| 32 Hz | ~4.8 rad/s |
| 64 Hz | ~2.4 rad/s |
| 128 Hz | ~1.2 rad/s |

At 32 Hz on a 42 cpr encoder, the quantization noise on `actual_velocity` is
large enough to trip the Elmo drive's internal speed-tracking limit during a
position profile (CSP mode), even when the *actual* mechanical velocity is
well within bounds. 64 Hz is sufficient headroom for the Elmo's default
tracking thresholds; higher rates further suppress quantization noise.

This applies equally to higher-resolution encoders — the noise is just less
visible because each count corresponds to a smaller velocity step.
