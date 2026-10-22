# Pineapple Hardware Interface
DDS hardware interface for CSL wheel-biped robot
## Overview
### Hardware
Pineapple V2:
- Joint motors: Damiao DM8006 DM8009P
- Wheel motors: Damiao DM6006
- IMU: Xsens Mti320
Pineapple V3:
- Joint motors: Damiao DM4340 DM4340P DM6248P
- Wheel motors: Damiao DM6006
- IMU: Xsens Mti320

## Requirement
- USB2CANFD firmware version (app v1.0.0.1)
- Damiao motor firmware version (v3 or v4)
- MTi-320 (IMU): verify the detected baud rate with `imu_probe` (tested V3: 2000000 baud). The interface configures quaternion, gyro and acceleration itself (100 Hz by default).
- Damiao motors: check that every motor replies within 2 ms with `motor_probe` (see [Motor response check](#motor-response-check)).

## Build
1. Follow this repo to install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).
    ```
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
    sudo apt-get update
    sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
    sudo make install
    ```
2. Set up IMU lib
    ```
    cd ~/pineapple_hardware_interface/xspublic
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

A 100 Hz IMU can still arrive in delayed bursts because the FTDI USB adapter
buffers data for its latency timer (default 16 ms). On V3, lowering it to 1 ms
reduced the maximum observed callback gap from 50.7 to 10.5 ms. With the default
16 ms the IMU startup check may fail, so set one of the options below.

Stop the hardware interface first and replace `ttyUSB0` with the IMU's port.

### Option A: sysfs latency_timer

```sh
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo ./build/imu_probe --measure
```

Expect readback `1` and `PASS`.

### Option B: setserial low_latency

```sh
sudo apt install setserial
sudo setserial /dev/ttyUSB0 low_latency
setserial -a /dev/ttyUSB0
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo ./build/imu_probe --measure
```

`setserial -a` should list `low_latency` under `Flags`, the timer should read `1`
and the probe should report `PASS`. To undo it:

```sh
sudo setserial /dev/ttyUSB0 ^low_latency
```

With the `ftdi_sio` driver both options program the same 1 ms timer into the
adapter, so use only one. While `low_latency` is set the timer stays at 1 ms
regardless of what is written to `latency_timer`. Other USB serial drivers
(CP210x, CH341) accept the flag but do not change any hardware timer.

### Make it persistent

Both settings reset when the IMU is reconnected or the computer reboots. Identify
the IMU adapter (this command only reads IDs):

```sh
udevadm info --attribute-walk --name=/dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
```

Replace `VID`, `PID` and `SERIAL` with the IMU adapter's values from the same
parent block. Do not use the USB hub's IDs or apply this to every FTDI adapter.
Install one of the two rules:

```sh
# Option A
sudo tee /etc/udev/rules.d/99-pineapple-imu-latency.rules >/dev/null <<'RULE'
ACTION=="add", SUBSYSTEM=="usb-serial", ATTRS{idVendor}=="VID", ATTRS{idProduct}=="PID", ATTRS{serial}=="SERIAL", ATTR{latency_timer}="1"
RULE

# Option B
sudo tee /etc/udev/rules.d/99-pineapple-imu-latency.rules >/dev/null <<'RULE'
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="VID", ATTRS{idProduct}=="PID", ATTRS{serial}=="SERIAL", RUN+="/usr/bin/setserial /dev/%k low_latency"
RULE
```

```sh
sudo chmod 0644 /etc/udev/rules.d/99-pineapple-imu-latency.rules
sudo udevadm control --reload-rules
```

Reloading does not apply the rule to an adapter that is already connected. Unplug
and reconnect the IMU, then verify (the port number may change):

```sh
udevadm settle
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo ./build/imu_probe --measure
```

## IMU startup check

When any config sets `have_imu: true`, the interface opens and configures the IMU
before any motor is enabled, then measures delivery for one second. It requires
each output to arrive within ±10% of its configured rate, the newest sample and
every gap between samples to be under the profile limit (20 ms for 100 Hz), and no
invalid quaternions. The measured line is printed as:

```
[imu] q/gyro/accel_hz=100/100/100 host_age_ms=... max_field_gap_ms=... ...
```

If the IMU is missing, cannot be configured, or fails the check, the interface
prints the reason and exits with code 1. No motor is enabled in that case. Fix
the IMU (see the latency section above) and start it again.

`imu_probe --measure` runs the same check ten times without touching the motors.
Stop the hardware interface first so only one process owns the IMU.

## Motor response check

`motor_probe --measure` measures how long each Damiao motor takes to answer a
command. It enables the motors in one config, sends each motor a zero-torque MIT
command (`kp=0 kd=0 tau=0`) and waits for its feedback frame, one motor after
another, every 2 ms for 10 seconds. The motors are disabled when it finishes or on
Ctrl+C.

The joints go limp while it runs, so hang the robot up or support it first. Stop
the hardware interface too, because only one process can own the USB2CANFD.

```sh
cd ~/pineapple_hardware_interface/build
sudo ./motor_probe --measure ../config/config_v3.yaml
```

It prints one line per second, then a table per motor:

```
[motor] t=1s rtt_ms mean=... max=... cycle_ms max=... lost=0
...
idx can_id type         sent  lost   min_ms  mean_ms   p50_ms   p99_ms   max_ms err t_mos
0   0x5    DM4340       ...
bus cycle (8 motors) mean_ms=... p99_ms=... max_ms=...
PASS (limit: no lost replies, max rtt < 2.0 ms)
```

- `rtt` is the time from sending the command to receiving that motor's feedback
  frame.
- `lost` counts commands with no reply within 10 ms.
- `bus cycle` is the time to get replies from all motors in one round.
- `err` and `t_mos` are the motor's last error code and MOS temperature.

The probe reports `PASS` and exits 0 when no reply is lost and every motor's max
rtt is under 2 ms, the interface's loop period. Otherwise the failing motors are
marked `FAIL` and it exits 1.


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
