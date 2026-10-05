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
