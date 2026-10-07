"""Offline regression checks; standard library only."""

import tempfile
from pathlib import Path

from summarize_imu_recording import audit, pair_serial_reads, percentile


def test_integrity_and_thresholds():
  with tempfile.TemporaryDirectory() as directory:
    prefix = Path(directory) / "test"
    csv_path = Path(str(prefix) + ".csv")
    csv_path.write_text(
      "host_monotonic_s,packet_counter,sample_time_fine_ticks,field_mask,qw,qx,qy,qz,gx,gy,gz,ax,ay,az\n"
      "10,65535,1000,7,1,0,0,0,0,0,0,0,0,9.8\n"
      "10.002,0,1020,7,1,0,0,0,0,0,0,0,0,9.8\n"
      "10.010,2,1100,7,1,0,0,0,0,0,0,0,0,9.8\n"
    )
    Path(str(prefix) + ".log").write_text("rows=3 capture_dropped=0 interrupted=0\n")
    m = audit(prefix)
    assert m["rows"] == 3 and m["missing_packets"] == 1
    assert m["duplicate_counters"] == 0
    assert m["fields"]["gyro"]["gaps_ge_threshold"] == 1
    assert abs(m["fields"]["gyro"]["host_gap_ms"]["max"] - 8) < 1e-8
    Path(str(prefix) + ".log").write_text("rows=4 capture_dropped=0 interrupted=0\n")
    assert any("rows disagree" in w for w in audit(prefix)["warnings"])
    csv_path.write_text(csv_path.read_text() + "broken\n")
    try:
      audit(prefix)
    except ValueError:
      pass
    else:
      raise AssertionError("Malformed CSV must fail")


def test_percentile():
  assert percentile([], 95) is None
  assert percentile([1, 3], 50) == 2
  assert percentile([7], 99) == 7


def test_read_capture_edges():
  dt, partial = pair_serial_reads([2, 4, 6], [1, 3, 5])
  assert dt == [1000, 1000] and partial == 2
  assert pair_serial_reads([1, 3], [2, 4]) == ([1000, 1000], 0)
  try:
    pair_serial_reads([1, 2], [3, 4])
  except ValueError:
    pass
  else:
    raise AssertionError("Ambiguous interior pairs must not be accepted")


if __name__ == "__main__":
  test_read_capture_edges()
  test_integrity_and_thresholds()
  test_percentile()
  print("Offline summarizer tests passed")
