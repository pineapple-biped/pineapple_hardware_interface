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
