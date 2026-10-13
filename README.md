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
- MTi-320 (IMU): verify the detected baud rate with `imu_probe` (tested V3: 2000000 baud). The interface configures quaternion, gyro and acceleration itself (100 Hz by default).

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

## IMU output profiles

`PINEAPPLE_IMU_PROFILE` selects the IMU output rates (quaternion/gyro/acceleration):

| Profile | Rates (Hz) | Minimum baud |
|---|---|---|
| `baseline` (default) | 100/100/100 | 115200 |
| `fast` | 100/1000/1000 | 2000000 |
| `fast500` | 100/500/500 | 2000000 |
| `fast250` | 100/250/250 | 2000000 |

The interface refuses a profile the detected baud rate cannot carry and never
changes the baud rate itself. Pass the same variable to the probe and the
interface:

```sh
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/imu_probe --measure
sudo env PINEAPPLE_IMU_PROFILE=fast ./build/pineapple_hardware_interface ../config/config_v3.yaml
```

## USB receive fix

The vendor USB2CANFD library discards bytes that libusb returns together with a
timeout. The interface replaces its two receive threads with a loop that keeps
those bytes, carries incomplete packets across reads and checks each packet's
length, CRC and tail before decoding. CMake builds a local copy of the vendor
archive with only those two symbols weakened; the original archives are not
modified. On clean shutdown each endpoint prints a `USB_RX_FIXED` line with
`partial_timeouts`, `recovered_bytes`, `frames`, `rejected_bytes` and
`pending_bytes`. This fix has not yet been validated on hardware.

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
