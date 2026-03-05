#!/usr/bin/env bash
set -euo pipefail

# Defaults (override via env vars or CLI flags)
FASTCAT_YAML="${FASTCAT_YAML:-test_unit/test_delayed_actuator_cmd_yamls/tnet_testbed_config.yaml}"
BIN_PATH="${BIN_PATH:-../build/bin/fastcat_delayed_actuator_cmd}"

MIN_HZ="${MIN_HZ:-100}"
MAX_HZ="${MAX_HZ:-2500}"
STEP_HZ="${STEP_HZ:-100}"

DELAY_SEC="${DELAY_SEC:-5.0}"
TARGET_POSITION="${TARGET_POSITION:-10.0}"
PROFILE_VELOCITY="${PROFILE_VELOCITY:-1.0}"
PROFILE_ACCEL="${PROFILE_ACCEL:-1.0}"
ENABLE_TELEMETRY="${ENABLE_TELEMETRY:-1}"

usage() {
  cat <<'EOF'
Usage:
  run_delayed_actuator_cmd_sweep.sh [options]

Options:
  --yaml <path>          YAML config path (default: $FASTCAT_YAML)
  --bin <path>           Binary path (default: $BIN_PATH)
  --min-hz <int>         Sweep start frequency, inclusive (default: $MIN_HZ)
  --max-hz <int>         Sweep end frequency, inclusive (default: $MAX_HZ)
  --step-hz <int>        Sweep step (default: $STEP_HZ)
  --delay <float>        Delay seconds (default: $DELAY_SEC)
  --position <float>     Target position (default: $TARGET_POSITION)
  --velocity <float>     Profile velocity (default: $PROFILE_VELOCITY)
  --accel <float>        Profile accel (default: $PROFILE_ACCEL)
  --telemetry <0|1>      Enable telemetry argument passed to binary (default: $ENABLE_TELEMETRY)
  --help                 Show this help

Environment overrides:
  FASTCAT_YAML, BIN_PATH, MIN_HZ, MAX_HZ, STEP_HZ,
  DELAY_SEC, TARGET_POSITION, PROFILE_VELOCITY, PROFILE_ACCEL, ENABLE_TELEMETRY
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --yaml)
      FASTCAT_YAML="$2"
      shift 2
      ;;
    --bin)
      BIN_PATH="$2"
      shift 2
      ;;
    --min-hz)
      MIN_HZ="$2"
      shift 2
      ;;
    --max-hz)
      MAX_HZ="$2"
      shift 2
      ;;
    --step-hz)
      STEP_HZ="$2"
      shift 2
      ;;
    --delay)
      DELAY_SEC="$2"
      shift 2
      ;;
    --position)
      TARGET_POSITION="$2"
      shift 2
      ;;
    --velocity)
      PROFILE_VELOCITY="$2"
      shift 2
      ;;
    --accel)
      PROFILE_ACCEL="$2"
      shift 2
      ;;
    --telemetry)
      ENABLE_TELEMETRY="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ ! -f "$FASTCAT_YAML" ]]; then
  echo "YAML file not found: $FASTCAT_YAML" >&2
  exit 1
fi
if [[ ! -x "$BIN_PATH" ]]; then
  echo "Binary is not executable: $BIN_PATH" >&2
  exit 1
fi
if ! [[ "$MIN_HZ" =~ ^[0-9]+$ && "$MAX_HZ" =~ ^[0-9]+$ && "$STEP_HZ" =~ ^[0-9]+$ ]]; then
  echo "Frequency values must be integers." >&2
  exit 1
fi
if (( MIN_HZ <= 0 || MAX_HZ <= 0 || STEP_HZ <= 0 )); then
  echo "Frequency values must be > 0." >&2
  exit 1
fi
if (( MIN_HZ > MAX_HZ )); then
  echo "--min-hz must be <= --max-hz." >&2
  exit 1
fi
if ! [[ "$ENABLE_TELEMETRY" =~ ^[01]$ ]]; then
  echo "--telemetry must be 0 or 1." >&2
  exit 1
fi

yaml_backup="$(mktemp "${TMPDIR:-/tmp}/fastcat_yaml_backup.XXXXXX")"
yaml_tmp=""
cp "$FASTCAT_YAML" "$yaml_backup"

cleanup() {
  if [[ -n "$yaml_tmp" && -f "$yaml_tmp" ]]; then
    rm -f "$yaml_tmp"
  fi
  if [[ -f "$yaml_backup" ]]; then
    cp "$yaml_backup" "$FASTCAT_YAML"
    rm -f "$yaml_backup"
  fi
}
trap cleanup EXIT

set_loop_rate_hz() {
  local hz="$1"
  yaml_tmp="$(mktemp "${TMPDIR:-/tmp}/fastcat_yaml_rate.XXXXXX")"
  awk -v hz="$hz" '
    BEGIN { replaced = 0 }
    {
      if (!replaced && $0 ~ /^[[:space:]]*target_loop_rate_hz:[[:space:]]*/) {
        indent = ""
        if (match($0, /^[[:space:]]*/)) {
          indent = substr($0, RSTART, RLENGTH)
        }
        print indent "target_loop_rate_hz: " hz
        replaced = 1
      } else {
        print $0
      }
    }
    END {
      if (!replaced) {
        exit 42
      }
    }
  ' "$FASTCAT_YAML" > "$yaml_tmp"

  mv "$yaml_tmp" "$FASTCAT_YAML"
  yaml_tmp=""
}

RUN_CMD=(sudo "$BIN_PATH")

echo "Sweep config:"
echo "  YAML:             $FASTCAT_YAML"
echo "  Binary:           $BIN_PATH"
echo "  Range:            ${MIN_HZ}..${MAX_HZ} Hz (step ${STEP_HZ})"
echo "  Cmd args:         $DELAY_SEC $TARGET_POSITION $PROFILE_VELOCITY $PROFILE_ACCEL $ENABLE_TELEMETRY"

for (( hz = MIN_HZ; hz <= MAX_HZ; hz += STEP_HZ )); do
  echo
  echo "=== Running ${hz} Hz ==="
  set_loop_rate_hz "$hz"

  if "${RUN_CMD[@]}" \
    "$DELAY_SEC" \
    "$TARGET_POSITION" \
    "$PROFILE_VELOCITY" \
    "$PROFILE_ACCEL" \
    "$ENABLE_TELEMETRY" \
    "$FASTCAT_YAML"; then
    echo "PASS ${hz} Hz"
  else
    rc=$?
    echo "FAIL ${hz} Hz (exit code ${rc}); stopping sweep early." >&2
    exit "$rc"
  fi
done

echo
echo "Sweep completed successfully."
