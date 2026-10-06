# IMU delivery fix and validation

Based on `v3-dev` at `6f5af7d`. Motor configuration, directions, offsets,
limits, motor control and deployment-policy gains are unchanged.

The hardware log `move_new_20261005_173234` showed approximately 21 fresh IMU
values/s, with a 50 ms median hold for quaternion, gyro and acceleration despite
500 Hz lowstate publication. That is not 100 Hz delivery. A normal 100 Hz sensor
will repeat across about five 500 Hz lowstate messages; repetition alone is not
an error.

Changes:

- Remove the five-packet application queue and update the latest sample directly
  from the SDK callback. No polling sleep, logging, or motor work in acquisition.
- Replace the sequence-counter snapshot of non-atomic memory with a mutex that
  protects both writes and copies. The previous implementation had a C++ data race.
- Request and verify the configured rate of all three outputs at 100 Hz. This
  is the default for the documented 115200-baud link. Do not request 1000 Hz
  gyro/acceleration on that link or increase rates without budgeting bandwidth.
- Print actual per-field arrival rates, host age, maximum callback gap, last
  device timestamp interval, missing packet count and invalid quaternion count.
- Add `imu_probe`, which opens/configures only the IMU. It never initializes CAN,
  enables motors, or creates DDS endpoints.

These changes remove known application buffering/race problems. They do not prove
that the device/USB/serial transport now delivers timely data. The prior recording
cannot distinguish device configuration from transport batching. No hardware
performance improvement is claimed until the probe and a new recording verify it.
No estimator or interpolation is used to hide stale samples.

## Build on the robot

Use the same checkout/build that will run on the robot; rebuilding a different
checkout does not update an existing executable. `git rev-parse --short HEAD`
records the source revision; `git status --short` shows local modifications.

```sh
make -C xspublic clean
make -C xspublic -j2
cmake -S . -B build
cmake --build build -j2 --target pineapple_hardware_interface imu_probe test_imu_delivery
ctest --test-dir build --output-on-failure
```

Stop the currently running hardware interface so it releases the IMU, then run:

```sh
sudo ./build/imu_probe --measure | tee imu_probe.log
```

The probe prints the configured rates then runs ten one-second delivery windows
after warmup. It returns nonzero for failed configuration or delivery checks.
This changes the IMU's measurement output configuration to 100 Hz, not motor
configuration. Do not run it alongside the regular interface.

Expected: `configured_hz=100` for all three outputs, `q/gyro/accel_hz` around
100/100/100, `last_device_interval_ms` around 10 for full samples, no increasing
missing packets and host ages usually below 10–15 ms. The probe's conservative
screen requires 90–110 Hz each window, sampled host age below 30 ms, and no
callback gap above 30 ms during the measurement. It is a delivery screen, not
certification of control stability or absolute sensor latency.

Interpretation:

- Configured rate other than 100: rejected at startup; inspect sensor output
  support/configuration before using the controller.
- Device timestamp intervals around 50 ms and received rate around 20 Hz:
  inspect device output and packet loss. Missing counters distinguish loss from
  genuinely slower sampling when counters are available.
- Device intervals around 10 ms but 50 ms callback gaps: transport/SDK batches
  data. Average 100 Hz is insufficient if samples arrive in bursts.
- Probe passes but the full interface does not: inspect full-process load,
  scheduling or a different/stale executable. Runtime diagnostics use the same
  callback and configuration code as the probe.

For serial USB adapters, inspect the adapter's sysfs `latency_timer` (if present)
and actual baud rate. Do not indiscriminately write sysfs settings: adapter type
and supported rates must be established first. Diagnostics currently report host
callback age; they cannot infer absolute device-to-host delay from unsynchronized
clocks. Device internal attitude filtering can add latency even with regular delivery.

After the probe passes, stop it, run the usual V3 interface command using your
existing robot config, and capture the diagnostic output plus a short new DDS
recording. Recheck the update cadence before interpreting a balancing test.

## Offline checks

The complete hardware executable and IMU-only probe compile locally. The CTest
suite exercises packet/timestamp rollover, dropped-packet accounting, missing
fields, invalid quaternions, and concurrent snapshot consistency. No physical
IMU or motors were accessed during development.

## Explicit baseline / fast output profiles

The default is still `baseline`: quaternion/gyro/acceleration at 100/100/100 Hz.
The opt-in `fast` profile requests 100/1000/1000 Hz, requires a detected baud
rate of at least 2,000,000, and verifies the actual output-configuration readback.
It does not change the baud rate, motor calibration, PD gains or policy.
Unsupported rates or insufficient baud are rejected; a requested IMU that fails
startup now prevents the full bridge from accepting motor commands instead of
continuing with identity orientation. A valid profile does not certify timely
physical delivery: the probe must still pass on the robot.

This branch includes the preceding IMU freshness and command-watchdog fixes.
Preserve the robot's existing motor IDs, offsets, directions, limits and USB
serial numbers when updating. Do not blindly replace a locally calibrated YAML
or pop a stash of older IMU source over these changes.

After rebuilding as above, stop the controller and hardware interface. Ensure
no other process owns the IMU. Verify the FTDI latency timer persists at `1`:

```sh
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# If it is not 1, use the device-specific persistent rule in README.md.
sudo env PINEAPPLE_IMU_PROFILE=baseline ./build/imu_probe --measure 2>&1 | tee imu_baseline.log
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/imu_probe --measure 2>&1 | tee imu_fast.log
```

The explicit `sudo env` matters: plain `sudo` may strip environment variables.
The probe configures only the sensor; it never connects to CAN or publishes DDS.
With fast mode, expect configured rates 100/1000/1000 and received rates within
10% of those values. The probe additionally requires per-field maximum callback
update gaps and sampled host ages below 20/5/5 ms, no added missing packets and
no invalid quaternions. Baseline per-field gap/age thresholds are 20/20/20 ms.
These thresholds are diagnostic targets, not claimed measurements or stability
guarantees. Mixed-rate packets can have several device timestamp spacings;
do not expect every packet to have a 10 ms interval in fast mode.

If fast fails, save its output and do not proceed to a balancing comparison.
Re-run the baseline probe to restore the baseline sensor configuration. Do not
relax thresholds just to obtain PASS. Increasing rate can reveal serial/SDK/CPU
bottlenecks, which must be measured.

After the fast probe exits successfully, the same profile must be selected for
normal bridge startup; otherwise the default resets all fields to 100 Hz:

```sh
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/pineapple_hardware_interface config/config_v3.yaml 2>&1 | tee interface_imu_fast.log
```

This last command starts the motor-capable hardware interface. Use the robot's
reviewed config and the existing supervised hardware procedure. Run the deployed
controller separately with its existing `use_ang_vel_filter: false`. Inspect the
bridge's per-field rates, ages and maximum gaps under full load; probe success
alone does not establish full-system delivery performance.

For a controlled comparison, record separate baseline and fast trials with the
same checkpoint, commanded motion, gains, pose and support conditions. Record
source commit, local diff, config hash and ONNX/checkpoint hash alongside MCAP.
Do not change wheel Kv at the same time. Include deploy status/meta/cmd_vel topics
where available; previous recordings omitted them, preventing policy attribution.

These changes do not yet add device timestamps/field sequences to the DDS schema
or instrument age at policy inference. The printed host age starts at the SDK
callback, so device filtering and earlier USB buffering are not included. Lowstate
still publishes at 500 Hz: a 1000 Hz gyro is sampled into that stream, rather than
all 1000 samples being forwarded. Latest-value delivery can reduce sample age,
but it is not an anti-alias filter or a zero-latency guarantee.

Offline validation: all hardware-interface/probe/test targets compile; the two
CTest suites pass. Tests cover profile selection, invalid-profile rejection,
rate mismatch, per-field stale/burst checks and existing delivery/watchdog cases.
The fast profile has not been validated on the physical MTi by this code change.

Baud validation uses the SDK's enum-to-numeric conversion. A diagnostic such as
`baud=2000000` followed by `detected 4107` indicates the original profile patch's
conversion bug; update and rebuild before probing. SDK baud enums are not baud
numbers. Regression tests cover acceptance at 2 Mbaud and rejection at low rates.
Clean Xsens objects before the first build after switching branches or machines:
the repository contains compiled objects that may belong to another architecture.

## Motor-free stationary and hand-motion recording

`imu_probe --measure --seconds 60 --record NEW.csv` records raw per-packet IMU
fields and SDK callback monotonic times, device SampleTimeFine ticks (0.1 ms),
packet counters and field-presence bits. Missing fields are NaN, never invented
or filled with a previous value. Quaternion values are recorded before the
bridge's normalization. This executable has no CAN/DDS/motor initialization.

Stop the existing controller/interface with the robot physically supported.
Do not rely on active balancing or motor torque to support it. Keep the IMU fixed
to the torso; gently rock the supported body rather than shaking cables or
striking the sensor. Do not run the full hardware interface for this test.

Update/rebuild the probe, confirm FTDI timer=1, then record two separate runs:

```sh
mkdir -p imu_recordings
stamp=$(date +%Y%m%d_%H%M%S)
set -o pipefail
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/imu_probe --measure --seconds 30 \
  --record "imu_recordings/static_${stamp}.csv" 2>&1 | tee "imu_recordings/static_${stamp}.log"

stamp=$(date +%Y%m%d_%H%M%S)
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/imu_probe --measure --seconds 60 \
  --record "imu_recordings/hand_motion_${stamp}.csv" 2>&1 | tee "imu_recordings/hand_motion_${stamp}.log"
```

For static: do not touch the robot for the whole 30 seconds.
For hand motion, use the printed BEGIN/elapsed time: 0–10 s still; 10–25 s gentle
pitch motion; 25–40 s gentle roll motion; 40–50 s gentle yaw motion if the fixture
allows it; 50–60 s still. These are operator instructions, not automated phase
labels. Note any deviations when sharing data. No violent/high-frequency shaking
is needed. Record the mounting orientation and whether the feet/wheels or frame
were supported. Do not interpret the moving trace's standard deviation as noise.

Wait for the final recording line. It reports row count, capture_dropped and
interrupted. Existing CSVs are rejected before device access. Capture buffers are
preallocated in RAM (bounded duration 1–120 seconds), with no disk writes in the
callback; the CSV is written on completion or graceful Ctrl-C/SIGTERM. Forced kill
or power loss can lose buffered data. Retain the log even if the delivery check
fails: the CSV remains useful, and failures/overflow must not be hidden.

Send both CSVs and both logs for analysis. Unlike one-second probe summaries,
these contain raw waveforms for separate orientation, gyro, acceleration and
timing plots. They still do not measure absolute internal filter latency or
motor/controller timing. Device-clock and host-clock gaps can reveal batching;
relative attitude/gyro lag under motion includes sensor fusion dynamics.

Offline plotting on a workstation with NumPy and Matplotlib (no robot connection):

```sh
python scripts/plot_imu_recording.py imu_recordings/static_TIMESTAMP.csv
python scripts/plot_imu_recording.py imu_recordings/hand_motion_TIMESTAMP.csv
```

## Investigating residual 2 ms batching and occasional long gaps

In the Oct 6 raw captures, device timestamps advance every 10 ms for orientation
and 0.9–1.0 ms for gyro/acceleration, without packet-counter gaps. Static gyro
arrival gaps are below 0.2 ms in 52.3% of cases and between 1.8–2.2 ms in 47.0%.
There are 34 gyro gaps >=5 ms, all between elapsed 27.03 and 29.99 seconds;
maximum 7.36 ms. Acceleration shows the same late disturbance. The corresponding
device gaps remain 1 ms. Hand-motion recording passed, maximum about 4.11 ms.

A concrete software source of batching exists in the bundled SDK:
`DataPoller::conjureUpWaitTime` waits 2 ms after reads smaller than 256 bytes;
`StandardThread::threadMain` implements that wait. `SerialCommunicator` otherwise
uses a nonblocking serial read; the parser drains queued messages together.
This is separate from our removed application queue and the FTDI latency timer.
It matches the regular 2 ms arrival pattern but does not by itself establish the
cause of the late 7.36 ms stalls. USB buffering and OS scheduling remain possible.

The recorder reserves a bounded vector before opening the IMU and writes CSV
only after capture stops. It does not reallocate within capacity. Reserving
memory does not guarantee all pages are resident: page faults, SDK allocations,
and process scheduling still need measurement. The probe now reports once-per-
second process CPU, page-fault and context-switch deltas. These are process-wide,
not receive-thread attribution; they provide correlation rather than proof.

An opt-in comparison reduces the SDK small-read wait to 1 ms without busy-spin,
realtime priorities, changed rates/gains, or relaxed failure thresholds. The
stock behavior is retained unless `PINEAPPLE_XSENS_LOW_LATENCY=1` is set.
This affects the bundled SDK; it MUST be rebuilt before comparing:

```sh
git pull --ff-only
make -C xspublic clean && make -C xspublic -j2 &&
cmake --build build -j2 &&
ctest --test-dir build --output-on-failure
```

With the controller and hardware interface stopped and the robot supported,
run both IMU-only cases, otherwise identical. Keep the FTDI timer at 1 ms.
No CAN, DDS or motor code is initialized by these probe commands:

```sh
mkdir -p imu_recordings
set -o pipefail
stamp=$(date +%Y%m%d_%H%M%S)
sudo env PINEAPPLE_IMU_PROFILE=fast PINEAPPLE_XSENS_LOW_LATENCY=0 \
  ./build/imu_probe --measure --seconds 30 \
  --record "imu_recordings/poll2_${stamp}.csv" 2>&1 | tee "imu_recordings/poll2_${stamp}.log"

stamp=$(date +%Y%m%d_%H%M%S)
sudo env PINEAPPLE_IMU_PROFILE=fast PINEAPPLE_XSENS_LOW_LATENCY=1 \
  ./build/imu_probe --measure --seconds 30 \
  --record "imu_recordings/poll1_${stamp}.csv" 2>&1 | tee "imu_recordings/poll1_${stamp}.log"
```

Keep the robot untouched throughout; note any other programs launched during the
run. Send all CSV/log files. Compare the arrival-gap distribution, >=5 ms events,
device gaps, CPU cost and fault/scheduling counters. A faster mean rate alone is
not success. If the regular 2 ms bands shrink but long gaps remain, treat those
as a separate host/transport problem. USB/kernel traces would then be needed to
localize earlier delay; syscall tracing can itself perturb timing.

Offline: library and full-interface/probe builds pass, as do both CTest suites.
No physical low-latency-mode result has been measured here. Do not label this as
a verified robot-side latency fix until paired recordings support it.
