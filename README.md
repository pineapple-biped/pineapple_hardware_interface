# Pineapple Hardware Interface
DDS hardware interface for CSL wheel-biped robot
## Overview
### Hardware
- Joint motors: Damiao DM8006 DM8009P
- Wheel motors: Damiao DM6006
- IMU: Xsens Mti320

## Requirement
- USB2CANFD firmware version (app v1.0.0.1)
- Damiao motor firmware version (v3 or v4)
- MTi-320: verify the detected baud rate with `imu_probe` (tested V3: 2000000 baud). The interface configures quaternion, gyro and acceleration at 100 Hz.

## Build
1. Follow this repo to install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).
2. Set up IMU lib
    ```
    cd ~/pineapple_hardware_interface/src/xspublic
    make clean
    make
    sudo usermod -G dialout -a $USER
    ```
3. Set up motor lib
    ```
    sudo apt install libusb-1.0-0
    sudo nano /etc/udev/rules.d/99-usb.rules
    ```
    add following line
    ```
    SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    ```
    cd ~/pineapple_hardware_interface
    mkdir build
    cd build
    cmake ..
    make
    ```
## IMU USB latency (FTDI adapters)

A 100 Hz sensor can still arrive in delayed bursts. On V3, changing the FTDI
latency timer from 16 to 1 ms reduced the maximum observed callback gap from
50.7 to 10.5 ms. This verifies delivery timing, not total sensor/filter latency.

Stop the hardware interface, substitute the IMU's actual port if different, then:

```sh
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo ./build/imu_probe --measure
```

Expect readback `1`, approximately `100/100/100` Hz and `PASS`. The probe does
not initialize motors. This setting may reset after reconnecting or rebooting.

For persistence, identify the IMU adapter (**this command only reads IDs**):

```sh
udevadm info --attribute-walk --name=/dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
```

Create `/etc/udev/rules.d/99-pineapple-imu-latency.rules` with the following line,
replacing `VID`, `PID` and `SERIAL` with the IMU USB device's values from the same
parent block. Do not use the USB hub's IDs or apply this to every FTDI adapter.

Replace the three placeholders below before running this command:

```sh
sudo tee /etc/udev/rules.d/99-pineapple-imu-latency.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb-serial", ATTRS{idVendor}=="VID", ATTRS{idProduct}=="PID", ATTRS{serial}=="SERIAL", ATTR{latency_timer}="1"
EOF
sudo chmod 0644 /etc/udev/rules.d/99-pineapple-imu-latency.rules
sudo udevadm control --reload-rules
```

With the hardware interface stopped, unplug and reconnect the IMU, then verify
(replace `ttyUSB0` if its port number changed):

```sh
udevadm settle
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo ./build/imu_probe --measure
```

The timer must read `1` and the probe should report `PASS`. Reloading the rules
alone does not apply them to an already-connected adapter. If no unique serial is present,
use a device-specific rule rather than omitting the selector blindly.
See [IMU delivery diagnostics](docs/IMU_DELIVERY.md) if the probe fails.

## Useage

Each config file describes one platform on its own USB2CANFD device. All motor
parameters (device serial number, CAN IDs, motor types, offsets, directions,
position limits) live in the config, so the same binary runs every robot.

### 1. Scan USB2CANFD serial numbers

```
cd ~/pineapple_hardware_interface/build
sudo ./scan_canfd_sn
```

### 2. Fill each SN into the matching config

Set the `dev_sn` field in `config/config.yaml` (wheel biped) and
`config/config_arm.yaml` (arm).

### 3. Run the hardware interface

Single platform:

```
cd ~/pineapple_hardware_interface/build
sudo ./pineapple_hardware_interface                           # v2 pineapple (config.yaml)
sudo ./pineapple_hardware_interface ../config/config_v3.yaml  # v3 pineapple
sudo ./pineapple_hardware_interface ../config/config_arm.yaml # 6-DOF pineapple arm
```

Whole body (wheel biped + arm, two USB2CANFD devices): pass both configs.
The joint index order in LowCmd/LowState follows the argument order —
wheel first is the convention:

```
sudo ./pineapple_hardware_interface ../config/config.yaml ../config/config_arm.yaml
# joints 0-7 = wheel biped, joints 8-13 = arm
```

## Command-timeout watchdog

Branch `fix/command-watchdog` includes the IMU-freshness fixes and a latched
command stop. Build and run the offline tests before installing on the robot:

```sh
git fetch origin
git switch fix/command-watchdog
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

After the first valid `rt/lowcmd` frame, **100 ms without another accepted frame**
latches the bridge off. A separate thread checks every 2 ms and repeatedly sends
CAN motor-disable (`0xFD`) every 20 ms while latched. These intervals are scheduling
targets, not a hard real-time guarantee or acknowledgment from the motors. Late
commands cannot revive an expired session. Fault recovery cannot clear/enable
motors after the latch. Stop the controller, resolve the cause and restart the
bridge to rearm. Switching controllers with a gap over 100 ms also requires restart.

Invalid/nonfinite command fields or gains outside the MIT encoder range also latch
off before any motor in that frame is commanded. This is not full per-joint command
validation. Normal SIGINT/SIGTERM shutdown requests disable as well. The watchdog
waits for the first command at startup; it is not a startup interlock. Existing
motor initialization/enable behavior is unchanged.

**Hardware acceptance test (not yet performed):** use a rigidly supported base,
clear wheels/legs and an independent emergency stop. With a known bounded command
publisher running, suspend/stop only that publisher while leaving the bridge
running. Check the watchdog message and confirm motor feedback reports disabled
and the last target is no longer driven. Resume the publisher and verify motors
remain disabled. Stop it again before restarting the bridge. Record the command
gap and motor response; the offline test does not measure actual CAN stop latency.
Disabling removes support torque: do not perform this test while balancing freely.

Only confirm the sysid collector's `--watchdog-confirmed` after this physical test.
The host watchdog protects publisher/network loss while the bridge and CAN remain
operational. It cannot protect against a killed/frozen bridge, host power loss,
USB/CAN failure or a frozen publisher continually retransmitting commands. Those
need an independently configured and tested **motor firmware communication timeout**
and emergency stop. The driver exposes a `TIMEOUT` register, but its units and
supported firmware behavior have not been verified; this change does not write it.
DDS receipt time is used, not a synchronized sensor/publisher clock.

## Optional faster gyro and acceleration

Default IMU output remains 100/100/100 Hz. On the verified 2 Mbaud link, an
explicit `PINEAPPLE_IMU_PROFILE=fast` requests quaternion/gyro/acceleration at
100/1000/1000 Hz, with configuration readback and per-field timing checks.
Start with the IMU-only probe, with the normal interface stopped:

```sh
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/imu_probe --measure
```

See [profile validation and full-interface commands](docs/IMU_DELIVERY.md#explicit-baseline--fast-output-profiles).
The full interface must receive the same environment variable; otherwise it
reconfigures baseline rates. Actual high-rate delivery remains to be tested on
the robot. Motor configuration and gains are unchanged.

For motor-free raw data collection, `imu_probe --measure --seconds 60 --record NEW.csv`
records individual IMU packets, timestamps and field-presence flags. See the
[stationary/hand-motion recording protocol](docs/IMU_DELIVERY.md#motor-free-stationary-and-hand-motion-recording).
The controller and normal hardware interface must remain stopped.

### Diagnose intermittent IMU delivery (motors off)

The 120-second FAST/1 ms test still showed intermittent 7.42 ms gyro arrival
intervals despite continuous packet counters and regular device timestamps.
A short passing test is not proof that stalls are eliminated.

On `fix/imu-rate-profiles`, rebuild the modified SDK and probe (cleaning the SDK
also removes any checked-in objects built for another CPU architecture):

```bash
cd ~/pineapple_hardware_interface
git pull --ff-only
make -C xspublic clean && make -C xspublic -j2 &&
  cmake --build build -j2 && ctest --test-dir build --output-on-failure
```

Stop the controller and hardware interface. Confirm the adapter's
`latency_timer` reads `1`. The following probe opens only the IMU; it does not
initialize CAN, DDS, or motors. Leave the supported robot stationary for this
delivery test; motion is not required to reproduce the stalls.

```bash
mkdir -p imu_recordings
stamp=$(date +%Y%m%d_%H%M%S)
sudo env PINEAPPLE_IMU_PROFILE=fast PINEAPPLE_XSENS_LOW_LATENCY=1 \
  ./build/imu_probe --measure --seconds 120 \
  --record "imu_recordings/trace_${stamp}.csv" \
  --trace "imu_recordings/trace_${stamp}.trace.csv" \
  > "imu_recordings/trace_${stamp}.log" 2>&1
```

Upload all three files, including when the probe reports FAIL. Existing CSVs
are never overwritten. Tracing is opt-in, allocates roughly 94 MB at 120 s
before device startup, and writes after SDK shutdown. It uses a bounded,
preinitialized event buffer; `[trace] dropped` must be zero for a complete
trace. The sensor recorder has a separate drop count. Tracing itself adds
measurement overhead; compare against the previous untraced recording.

Trace times use the same host monotonic clock as the sensor CSV. `source`
identifies a parser or callback instance, not a sensor timestamp. Events can
be out of time order across threads: sort by time for visualization.

- `read_begin/end`: SDK stream read entry/return; end value is byte count.
  A long interval between calls points toward poller wakeup/scheduling;
  regularly attempted empty reads followed by a burst point upstream of parsing.
  These markers cannot distinguish USB buffering from sensor transport alone.
- `enqueue/dequeue`: byte chunk pushed/popped under the parser queue mutex;
  pair FIFO per parser source. First/last capture chunks may be unmatched.
- `parse_begin/end`: raw-byte parsing; values are bytes and decoded message count.
- `dispatch_begin/end`: message handling (message ID), enclosing downstream callbacks.
- `callback_begin/end`: shared IMU snapshot update; begin value is packet counter,
  or -1 if unavailable. Match counters to sensor CSV with wraparound accounted for.
- `read_error`: SDK read return code on unsuccessful reads.

This diagnostic does not change gains, filtering, sensor rates, scheduling
priorities, or motor commands, and does not measure absolute sensor/filter latency.

### Controlled 500 Hz USB delivery comparison

`PINEAPPLE_IMU_PROFILE=fast500` requests quaternion 100 Hz and gyro/acceleration
500 Hz. It retains the 2 Mbaud requirement and the same delivery limits as
`fast` (orientation 20 ms, gyro/acceleration 5 ms). `baseline` remains the
default and `fast` remains 100/1000/1000 Hz. The SDK validates sensor output
configuration readback: unsupported rates fail rather than silently falling back.

This is an IMU-only diagnostic, not a controller deployment recommendation.
The 1000 Hz USB trace showed 6.926 ms gaps between payload completions and
occasional full 512-byte transfers. Reducing traffic tests whether transfer
batching contributes. Keep the USB latency timer at 1 ms and
`PINEAPPLE_XSENS_LOW_LATENCY=1`; change only the requested gyro/acceleration rate.
Run the same 120-second probe/trace with simultaneous 150-second usbmon capture.

After pulling `fix/imu-rate-profiles`, rebuild and test:

```bash
cmake --build build -j2 && ctest --test-dir build --output-on-failure
```

The SDK has not changed in this profile-only update; the previous trace-capable
SDK build is required. With the hardware interface/controller stopped, the
supported robot stationary, and usbmon already enabled, run:

```bash
mkdir -p imu_recordings
stamp=$(date +%Y%m%d_%H%M%S)
prefix="imu_recordings/usb500_${stamp}"
sudo -v
# The verified adapter is currently on bus 2. Recheck lsusb if reconnected.
sudo timeout 150 cat /sys/kernel/debug/usb/usbmon/2u \
  > "${prefix}.usbmon.txt" 2> "${prefix}.usbmon.log" &
usb_capture_pid=$!
sleep 1
if sudo env PINEAPPLE_IMU_PROFILE=fast500 PINEAPPLE_XSENS_LOW_LATENCY=1 \
  ./build/imu_probe --measure --seconds 120 \
  --record "${prefix}.csv" --trace "${prefix}.trace.csv" \
  > "${prefix}.log" 2>&1
then
  echo "Probe passed."
else
  echo "Probe failed; retain the files for diagnosis."
fi
wait "$usb_capture_pid" || true
ls -lh "${prefix}"*
```

Upload all five files regardless of PASS/FAIL. Exit 124 from the USB timeout is
expected. Check that usbmon output is nonempty and its error log is empty.
Compare sample rate, tail gaps, packet continuity, and clock-offset variation;
a lower output rate alone does not establish lower absolute sensor latency.

### Controlled 250 Hz comparison

Use `PINEAPPLE_IMU_PROFILE=fast250` for orientation 100 Hz and gyro/acceleration
250 Hz, retaining 2 Mbaud, the 1 ms USB latency timer, and 1 ms SDK polling.
Rebuild with `cmake --build build -j2` and run CTest after pulling the profile.
Use the preceding simultaneous capture block with `fast500` replaced by
`fast250` and the output prefix `usb500_` replaced by `usb250_`. All five files
are needed. The probe remains motor-free; stop the controller and hardware
interface before capture.

At 250 Hz, the nominal gyro/acceleration interval is 4 ms. The existing 5 ms
host-gap criterion is deliberately unchanged, leaving only 1 ms of margin.
Assess excess gap above the nominal interval and delivery variation alongside
PASS/FAIL; lowering output rate does not necessarily improve sample freshness.
The SDK verifies the requested output-rate readback and fails if unsupported.
Existing profiles and the default are unchanged.

### Analyze recordings on the robot instead of uploading raw files

The offline summarizer uses only the Python standard library (works in the
robot's `rl` conda environment; no uv, NumPy, or plotting packages needed).
It opens recorded files only, never the IMU, CAN, DDS, or motors. Run it after
the probe and USB capture finish. Supply the filename prefix without an
extension; for example, using the capture shell's existing `prefix` variable:

```bash
python scripts/summarize_imu_recording.py --latest imu_recordings
```

Or supply an explicit path such as `imu_recordings/usb250_YYYYMMDD_HHMMSS`.
The script prints a short report to paste into chat and writes
`<prefix>.summary.json` with source hashes, field rates, gap percentiles,
ten worst sensor gaps, packet integrity, SDK stage durations, and per-endpoint
USB statistics. The JSON is small; raw files can stay on the robot. Re-running
replaces only that derived summary, never the recordings.

Missing optional files or count mismatches are reported. Malformed sensor or
trace rows fail the audit. USB text capture can itself omit events, so apparent
USB gaps are not automatically physical delays. Summaries support routine
comparisons; unusual failures may still require a targeted raw excerpt later.

### Readiness-based serial test at 500 Hz

`PINEAPPLE_XSENS_READINESS=1` opts into a POSIX serial reader that waits for
readability instead of polling an empty port and then sleeping. The wait is
bounded at 10 ms and returns as soon as bytes are available; 10 ms is not an
added sample delay. Successful readiness reads skip the poller's extra sleep.
Timeouts re-enter the bounded wait; errors retain a sleep to avoid spinning.
Other transports, explicit nonzero serial timeouts, and the default mode retain
previous behavior. This does not change USB transfer sizes or sensor filtering.

Rebuild **both the SDK and the application** before this test:

```bash
make -C xspublic clean && make -C xspublic -j2 &&
  cmake --build build -j2 && ctest --test-dir build --output-on-failure
```

Use the previous simultaneous USB/IMU capture block, naming the prefix
`ready500_${stamp}` and setting these three environment variables on `imu_probe`:

```bash
sudo env PINEAPPLE_IMU_PROFILE=fast500 PINEAPPLE_XSENS_LOW_LATENCY=1 \
  PINEAPPLE_XSENS_READINESS=1 \
  ./build/imu_probe --measure --seconds 120 \
  --record "${prefix}.csv" --trace "${prefix}.trace.csv" \
  > "${prefix}.log" 2>&1
```

Keep the robot supported and stationary, with the controller/hardware interface
stopped. The probe remains motor-free. After both captures finish, run the
robot-side summarizer and paste its output; no raw upload is normally needed.
In readiness mode `read_begin` to `read_end` includes the intentional wait for
new data, so a longer read duration alone is not evidence of a processing stall.
Compare arrival gaps and CPU against `fast500` with readiness unset. Physical
improvement must be established by that measurement, not by the offline tests.

### Sensor-only DDS and observation-age test

Do **not** use the normal hardware bridge for a motor-free test: its state
publisher polls motors, and the deployment controller initializes a LowCmd
publisher. Instead use the separate `imu_dds_probe` and `check_imu_dds.py` below.
The executable is not linked to the motor/CAN library. It only opens Xsens and
publishes diagnostic `LowState` on **`rt/imu_probe/lowstate`**, never the normal
`rt/lowstate` or any command topic. The Python process only subscribes.

Both processes run on the **same robot**, DDS domain 0, loopback `lo`, sharing
the Linux monotonic clock. The diagnostic message uses its otherwise unused
40-byte wireless-remote field for a signature, sequence and host timestamps;
it is incompatible with normal LowState consumers and must remain on the
diagnostic topic. Quaternion is wxyz and gyro is unfiltered, matching the
current deployment settings. Subscriber queue length is 10, publication is
500 Hz, and observation consumption is 50 Hz (matching `mjlab_v3_opt.yaml`
`simulation_dt=0.005`, `control_decimation=4`).

This measures an **isolated observation proxy**, not the complete controller:
it copies gyro and computes projected gravity with the deployment formula,
but does not run policy inference, state estimation, motor traffic, or control.
A passing result cannot validate those loads. Ages start at the SDK callback;
absolute sensor/filter latency is still unmeasured. Same-host execution is
required; do not run the receiver on a laptop.

After pulling `fix/imu-rate-profiles`, build (the readiness SDK from the previous
step must already be built), stop the controller/hardware interface/other IMU
probes, and use the robot's `rl` environment for Python DDS dependencies:

```bash
cmake -S . -B build && cmake --build build -j2 &&
  ctest --test-dir build --output-on-failure
conda activate rl
mkdir -p imu_recordings
stamp=$(date +%Y%m%d_%H%M%S)
prefix="imu_recordings/dds500_${stamp}"
sudo -v
sudo env PINEAPPLE_IMU_PROFILE=fast500 PINEAPPLE_XSENS_LOW_LATENCY=1 \
  PINEAPPLE_XSENS_READINESS=1 \
  ./build/imu_dds_probe --seconds 130 > "${prefix}.publisher.log" 2>&1 &
imu_publisher_pid=$!
python scripts/check_imu_dds.py --seconds 120 --output "${prefix}.summary.json"
wait "$imu_publisher_pid"
tail -n 3 "${prefix}.publisher.log"
```

Keep the robot supported and stationary. Paste the receiver summary and last
publisher lines. Output JSON must be a new path; raw files need not be uploaded.
A loopback multicast warning can occur even when unicast discovery succeeds.
If the receiver times out, inspect the publisher log; do not start the normal
controller as a workaround. The publisher exits after its time limit.

For development, `imu_dds_probe --synthetic` skips all device scanning/opening
and publishes synthetic IMU samples. A local 5-second synthetic check received
2500 DDS messages and consumed 250 observations without sequence loss. This
checks software wiring only; its timing is not evidence of robot performance.

The DDS receiver also records Python garbage-collection pause durations and
reports their overlap with the five worst callback delays and observation
intervals. This does not disable GC or change scheduling. A rare delay in this
allocating diagnostic can be caused by its own recording workload; do not
attribute it to the production controller without evidence. Correlated events
support a hypothesis but are not by themselves proof of causality.

To test whether those spikes come from cyclic GC, repeat the same motor-free
publisher/receiver test with a new output prefix and add `--gc-mode disabled`
to the receiver command:

```bash
python scripts/check_imu_dds.py --seconds 120 --gc-mode disabled \
  --output "${prefix}.summary.json"
```

The default is `--gc-mode normal`. Disabled mode collects before measurement,
suppresses automatic cyclic GC only for the bounded measurement, and restores
its previous state even on interruption or error. Reference counting remains
active. This changes only the diagnostic process, not the robot controller.
Compare maximum callback delay, observation interval, sequence gaps and GC
counts against normal mode with identical publisher settings. No rebuild is
needed for this Python-only change.

### Test the sysid recorder without motor commands

The diagnostic can exercise the actual `pineapple_sysid.collect.Recorder`
(queue, full LowState JSON serialization, MCAP compression and disk writes).
It imports only the recorder class; it never creates the collection `DDS`
transport or any command publisher. In your robot Python environment, install
`mcap` and `numpy` if missing (`python -m pip install 'mcap>=1.3,<2' numpy`).
Keep the same motor-free `imu_dds_probe` publisher running as above, choose a
fresh prefix, and run:

```bash
python scripts/check_imu_dds.py --seconds 120 --gc-mode disabled \
  --sysid-root "$HOME/pineapple-v3-sysid-stage" \
  --record-mcap "${prefix}.mcap" --output "${prefix}.summary.json"
```

Adjust `--sysid-root` to your checkout containing `pineapple_sysid/collect.py`.
The summary prints serialization/enqueue duration and queued/read-back message
counts; recording errors or a read-back count mismatch fail the test. Raw MCAP
stays on the robot; share only the printed summary. This is diagnostic IMU data
with placeholder motor fields, not a motor identification dataset. Although
stored under the recorder's `rt/lowstate` channel, its source is exclusively
`rt/imu_probe/lowstate`; the summary retains that source and synthetic flag.
The real command loop, motor feedback and full controller load are not tested.
