#!/usr/bin/env python3
"""Receive-only IMU DDS and 50 Hz observation-timing diagnostic, same host only."""

# ruff: noqa: B905 -- support robot Python versions older than 3.10
import argparse
import json
import math
import struct
import threading
import time
from pathlib import Path

from summarize_imu_recording import describe


def decode(message, now):
  magic, seq, published, qtime, gtime, atime = struct.unpack(
    "<4sI4d", bytes(message.wireless_remote)
  )
  if magic not in (b"IMD1", b"IMS1"):
    raise ValueError("Wrong diagnostic message signature")
  quat = tuple(float(v) for v in message.imu_state.quaternion)
  gyro = tuple(float(v) for v in message.imu_state.gyroscope)
  accel = tuple(float(v) for v in message.imu_state.accelerometer)
  stamps = (published, qtime, gtime, atime)
  if not all(math.isfinite(v) for v in stamps + quat + gyro + accel):
    raise ValueError("Nonfinite diagnostic field")
  if any(v <= 0 or v > now + 0.001 or now - v > 60 for v in stamps):
    raise ValueError("Invalid host timestamps; run both processes on the same robot")
  norm = sum(v * v for v in quat)
  if abs(norm - 1) > 0.05:
    raise ValueError("Invalid quaternion norm")
  # Mirrors deployment quat_rotate_inverse(q, [0,0,-1]); no gyro filter.
  w, x, y, z = quat
  gravity = (2 * w * y - 2 * x * z, -2 * w * x - 2 * y * z, 1 - 2 * w * w - 2 * z * z)
  return (seq, published, (qtime, gtime, atime), now, gyro, gravity, magic == b"IMS1")


def run(seconds, hz, output):
  # Importing these initializes no command publisher. Only a subscriber is created.
  from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
  from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

  ChannelFactoryInitialize(0, "lo")
  lock = threading.Lock()
  latest = None
  rows = []
  errors = []
  observations = []
  accepting = True

  def callback(message):
    nonlocal latest
    now = time.monotonic()
    try:
      sample = decode(message, now)
    except (ValueError, TypeError, struct.error) as exc:
      with lock:
        if accepting:
          errors.append(str(exc))
      return
    with lock:
      if accepting:
        latest = sample
        rows.append(sample)

  sub = ChannelSubscriber("rt/imu_probe/lowstate", LowState_)
  sub.Init(callback, 10)  # Same queue length as deployment subscriber.
  try:
    deadline = time.monotonic() + 30
    while latest is None and time.monotonic() < deadline:
      time.sleep(0.01)
    if latest is None:
      raise RuntimeError("No valid diagnostic data after 30 s: " + str(errors[:3]))
    with lock:
      rows.clear()
      errors.clear()
    begin = time.monotonic()
    next_tick = begin
    while time.monotonic() - begin < seconds:
      with lock:
        sample = latest
      now = time.monotonic()
      # Construct the IMU portion of an observation from a coherent snapshot.
      obs = sample[4] + sample[5]
      if not all(math.isfinite(v) for v in obs):
        raise RuntimeError("Invalid observation")
      observations.append((now, sample))
      next_tick += 1 / hz
      time.sleep(max(0, next_tick - time.monotonic()))
  finally:
    with lock:
      accepting = False
    sub.Close()
  if len(rows) < 2:
    raise RuntimeError("Insufficient DDS samples")
  seq = [r[0] for r in rows]
  steps = [(b - a) % (2**32) for a, b in zip(seq, seq[1:])]
  gaps = [(b[3] - a[3]) * 1000 for a, b in zip(rows, rows[1:])]
  result = dict(
    topic="rt/imu_probe/lowstate",
    same_host_loopback=True,
    synthetic=any(r[6] for r in rows),
    duration_s=seconds,
    observation_hz=hz,
    dds_samples=len(rows),
    observation_samples=len(observations),
    sequence_gaps=sum(max(0, v - 1) for v in steps if v < 2**31),
    duplicate_or_reordered=sum(v == 0 or v >= 2**31 for v in steps),
    invalid_messages=len(errors),
    error_examples=errors[:5],
    dds_arrival_gap_ms=describe(gaps),
    publish_to_callback_ms=describe([(r[3] - r[1]) * 1000 for r in rows]),
    observation_snapshot_hold_ms=describe([(t - r[3]) * 1000 for t, r in observations]),
    observation_tick_gap_ms=describe(
      [(b[0] - a[0]) * 1000 for a, b in zip(observations, observations[1:])]
    ),
    fields={},
  )
  for i, name in enumerate(["orientation", "gyro", "acceleration"]):
    result["fields"][name] = dict(
      sdk_callback_to_publish_age_ms=describe([(r[1] - r[2][i]) * 1000 for r in rows]),
      sdk_callback_to_dds_receive_age_ms=describe(
        [(r[3] - r[2][i]) * 1000 for r in rows]
      ),
      sdk_callback_to_observation_age_ms=describe(
        [(t - r[2][i]) * 1000 for t, r in observations]
      ),
    )
  result["limitation"] = (
    "Host age starts at SDK receipt, not sensor acquisition. Isolated DDS and observation proxy, no policy inference, motors, or full controller load."
  )
  with output.open("x") as f:
    json.dump(result, f, indent=2)
  print("IMU DDS SUMMARY (synthetic=" + str(result["synthetic"]) + ")")
  print("DDS samples:", len(rows), "observations:", len(observations))
  print(
    "Sequence gaps/reordered/invalid:",
    result["sequence_gaps"],
    result["duplicate_or_reordered"],
    len(errors),
  )
  for k in [
    "dds_arrival_gap_ms",
    "publish_to_callback_ms",
    "observation_snapshot_hold_ms",
    "observation_tick_gap_ms",
  ]:
    print(k, result[k])
  for name, v in result["fields"].items():
    print(
      name,
      "SDK receipt to observation age ms:",
      v["sdk_callback_to_observation_age_ms"],
    )
  print("JSON:", output)
  print(result["limitation"])
  return 1 if errors else 0


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("--seconds", type=int, default=120)
  p.add_argument("--observation-hz", type=float, default=50)
  p.add_argument("--output", type=Path, required=True)
  a = p.parse_args()
  if not 1 <= a.seconds <= 240 or not 1 <= a.observation_hz <= 500:
    p.error("seconds 1..240; observation-hz 1..500")
  if a.output.exists():
    p.error("Output exists; choose a new filename")
  if not a.output.parent.is_dir():
    p.error("Output directory does not exist")
  try:
    return run(a.seconds, a.observation_hz, a.output)
  except (RuntimeError, ValueError, OSError) as exc:
    p.exit(1, str(exc) + "\n")


if __name__ == "__main__":
  raise SystemExit(main())
