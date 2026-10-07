"""Metadata and observation checks; no DDS or hardware initialization."""

import math
import struct
from types import SimpleNamespace

from check_imu_dds import decode, gc_overlap_ms


def message(magic=b"IMD1", published=10.0, quat=(1.0, 0.0, 0.0, 0.0)):
  return SimpleNamespace(
    wireless_remote=struct.pack("<4sI4d", magic, 7, published, 9.99, 9.999, 9.998),
    imu_state=SimpleNamespace(
      quaternion=quat, gyroscope=(1, 2, 3), accelerometer=(0, 0, 9.81)
    ),
  )


def test_decode():
  sample = decode(message(), 10.001)
  assert sample[0] == 7 and sample[4] == (1, 2, 3) and sample[5] == (0, 0, -1)
  sample = decode(message(quat=(math.sqrt(0.5), 0, math.sqrt(0.5), 0)), 10.001)
  assert abs(sample[5][0] - 1) < 1e-12 and abs(sample[5][2]) < 1e-12
  for bad in [
    message(magic=b"BAD!"),
    message(published=20),
    message(quat=(0, 0, 0, 0)),
  ]:
    try:
      decode(bad, 10.001)
    except ValueError:
      pass
    else:
      raise AssertionError("Bad metadata or quaternion was accepted")


if __name__ == "__main__":
  test_decode()
  assert gc_overlap_ms(1, 2, [(1.5, 2.5, 2)]) == 500
  assert gc_overlap_ms(1, 2, [(3, 4, 0)]) == 0
  print("DDS diagnostic decode tests passed")
