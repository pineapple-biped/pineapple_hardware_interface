#!/usr/bin/env python3
"""Offline, standard-library-only IMU audit. Never opens hardware or DDS."""

# ruff: noqa: B905 -- retain compatibility with robot Python older than 3.10
import argparse
import csv
import hashlib
import json
import math
import re
import statistics
from array import array
from collections import Counter, defaultdict
from pathlib import Path


def percentile(values, p):
  if not values:
    return None
  x = sorted(values)
  f = (len(x) - 1) * p / 100
  i = int(f)
  return x[i] + (x[min(i + 1, len(x) - 1)] - x[i]) * (f - i)


def describe(values):
  return {
    k: percentile(values, p)
    for k, p in [("p50", 50), ("p95", 95), ("p99", 99), ("max", 100)]
  }


def audit(prefix):
  prefix = Path(prefix)
  files = {
    ext: Path(str(prefix) + ext)
    for ext in [".csv", ".log", ".trace.csv", ".usbmon.txt", ".usbmon.log"]
  }
  warnings = []
  result = {"recording": prefix.name, "files": {}, "warnings": warnings}
  for ext, path in files.items():
    if not path.exists():
      warnings.append("Missing " + ext)
      continue
    digest = hashlib.sha256()
    with path.open("rb") as f:
      for chunk in iter(lambda: f.read(1024 * 1024), b""):
        digest.update(chunk)
    result["files"][ext] = {"bytes": path.stat().st_size, "sha256": digest.hexdigest()}
  log = files[".log"].read_text() if files[".log"].exists() else ""
  match = re.search(r"rows=(\d+) capture_dropped=(\d+) interrupted=(\d+)", log)
  if not match:
    warnings.append("Missing recorder completion line")
  result["probe_result"] = (
    "PASS"
    if "PASS: host delivery" in log
    else "FAIL"
    if "FAIL: host delivery" in log
    else "UNKNOWN"
  )
  mode = re.search(r"\[imu\] readiness mode=(\w+)", log)
  result["readiness_mode"] = mode[1] if mode else "not logged (older probe)"
  result["configured_hz"] = dict(
    re.findall(r"output_id=(\d+) configured_hz=(\d+)", log)
  )
  cpu = [float(x) for x in re.findall(r"process_cpu_pct=([\d.]+)", log)]
  result["cpu_mean_percent_one_core"] = statistics.mean(cpu) if cpu else None
  result["capture_dropped"] = int(match[2]) if match else None
  result["interrupted"] = int(match[3]) if match else None
  fields = {
    name: {"times": array("d"), "ticks": array("d")}
    for name in ["orientation", "gyro", "acceleration"]
  }
  count = missing = duplicates = invalid = 0
  previous = previous_t = first = last = None
  with files[".csv"].open() as f:
    for row in csv.DictReader(f):
      t = float(row["host_monotonic_s"])
      counter = int(row["packet_counter"])
      if previous is not None:
        step = (counter - previous) % 65536
        missing += max(0, step - 1)
        duplicates += step == 0
        if t <= previous_t:
          raise ValueError("Sensor host timestamps do not strictly advance")
      previous, previous_t = counter, t
      if first is None:
        first = t
      last = t
      count += 1
      mask = int(row["field_mask"])
      for bit, name, columns in [
        (1, "orientation", ["qw", "qx", "qy", "qz"]),
        (2, "gyro", ["gx", "gy", "gz"]),
        (4, "acceleration", ["ax", "ay", "az"]),
      ]:
        if mask & bit:
          values = [float(row[c]) for c in columns]
          invalid += not all(math.isfinite(v) for v in values)
          fields[name]["times"].append(t)
          fields[name]["ticks"].append(float(row["sample_time_fine_ticks"]))
  if count < 2:
    raise ValueError("Fewer than two sensor rows")
  if match and count != int(match[1]):
    warnings.append("Sensor rows disagree with log: possibly incomplete upload")
  result.update(
    rows=count,
    duration_s=last - first,
    missing_packets=missing,
    duplicate_counters=duplicates,
    nonfinite_field_samples=invalid,
    fields={},
  )
  for name, values in fields.items():
    ts, ticks = values["times"], values["ticks"]
    gaps = array("d", ((b - a) * 1000 for a, b in zip(ts, ts[1:])))
    device = array("d", (((b - a) % (2**32)) * 0.1 for a, b in zip(ticks, ticks[1:])))
    threshold = 20 if name == "orientation" else 5
    worst = sorted(range(len(gaps)), key=gaps.__getitem__, reverse=True)[:10]
    result["fields"][name] = {
      "samples": len(ts),
      "hz": (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 else None,
      "host_gap_ms": describe(gaps),
      "device_gap_ms": describe(device),
      "threshold_ms": threshold,
      "gaps_ge_threshold": sum(v >= threshold for v in gaps),
      "worst_gaps": [{"end_s": ts[i + 1] - first, "gap_ms": gaps[i]} for i in worst],
    }
  if files[".trace.csv"].exists():
    groups = defaultdict(lambda: array("d"))
    counts = Counter()
    positive = array("d")
    with files[".trace.csv"].open() as f:
      for row in csv.DictReader(f):
        stage, source = row["stage"], row["source"]
        t = float(row["host_monotonic_s"])
        counts[stage] += 1
        groups[(stage, source)].append(t)
        if stage == "read_end" and int(row["value"]) > 0:
          positive.append(t)
    trace = {
      "events": sum(counts.values()),
      "stage_counts": dict(counts),
      "stages_ms": {},
    }
    tm = re.search(r"\[trace\] events=(\d+) dropped=(\d+)", log)
    trace["dropped"] = int(tm[2]) if tm else None
    if not tm or sum(counts.values()) != int(tm[1]):
      warnings.append("Trace count unverified or disagrees with log")
    for a, b in [
      ("read_begin", "read_end"),
      ("enqueue", "dequeue"),
      ("parse_begin", "parse_end"),
      ("dispatch_begin", "dispatch_end"),
      ("callback_begin", "callback_end"),
    ]:
      deltas = array("d")
      for stage, source in list(groups):
        if stage != a:
          continue
        aa, bb = sorted(groups[(a, source)]), sorted(groups[(b, source)])
        if len(aa) != len(bb):
          warnings.append("Unpaired trace stage: " + a)
          continue
        dt = [(y - x) * 1000 for x, y in zip(aa, bb)]
        if any(v < 0 for v in dt):
          warnings.append("Negative stage duration: " + a)
          continue
        deltas.extend(dt)
      trace["stages_ms"][a] = describe(deltas)
    positive = sorted(positive)
    trace["successful_read_gap_ms"] = describe(
      [(b - a) * 1000 for a, b in zip(positive, positive[1:])]
    )
    result["trace"] = trace
  if files[".usbmon.txt"].exists():
    # Track each bulk-IN endpoint separately; do not mix unrelated devices.
    endpoints = defaultdict(list)
    statuses = Counter()
    with files[".usbmon.txt"].open() as f:
      for line in f:
        parts = line.split()
        if len(parts) < 6 or parts[2] != "C" or not parts[3].startswith("Bi:"):
          continue
        t, status, length = int(parts[1]) * 1e-6, int(parts[4]), int(parts[5])
        if first <= t <= last:
          statuses[parts[3] + ":" + str(status)] += 1
        if status == 0 and length > 2:
          endpoints[parts[3]].append((t, length))
    result["usb"] = {"completion_status_counts": dict(statuses), "endpoints": {}}
    for endpoint, events in endpoints.items():
      events.sort()
      gaps = [
        (b[0] - a[0]) * 1000
        for a, b in zip(events, events[1:])
        if first <= b[0] <= last
      ]
      if not gaps:
        continue
      if events[0][0] > first + 0.05 or events[-1][0] < last - 0.05:
        warnings.append("USB endpoint does not cover full sensor interval: " + endpoint)
      result["usb"]["endpoints"][endpoint] = {
        "payload_gap_ms": describe(gaps),
        "gaps_ge5ms": sum(v >= 5 for v in gaps),
        "full512_completions": sum(first <= t <= last and n == 512 for t, n in events),
        "coverage_relative_s": [events[0][0] - first, events[-1][0] - first],
        "caution": "USB trace omissions can mimic gaps; this is not absolute latency.",
      }
  if files[".usbmon.log"].exists() and files[".usbmon.log"].stat().st_size:
    warnings.append("USB capture error log is nonempty; inspect it")
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
    "prefix", nargs="?", help="Recording path WITHOUT .csv/.log suffix"
  )
  parser.add_argument(
    "--latest", type=Path, help="Analyze newest sensor CSV in this directory"
  )
  args = parser.parse_args()
  if bool(args.prefix) == bool(args.latest):
    parser.error("Supply a prefix OR --latest DIRECTORY")
  if args.latest:
    candidates = [
      p
      for p in args.latest.glob("*.csv")
      if not p.name.endswith(".trace.csv") and p.with_suffix(".log").exists()
    ]
    if not candidates:
      parser.error("No sensor CSV with a matching log found")
    args.prefix = str(max(candidates, key=lambda p: p.stat().st_mtime).with_suffix(""))
  try:
    result = audit(args.prefix)
  except (ValueError, KeyError, OSError, TypeError) as exc:
    parser.exit(2, "Audit failed (possibly incomplete input): " + str(exc) + "\n")
  report = Path(str(args.prefix) + ".summary.json")
  report.write_text(json.dumps(result, indent=2) + "\n")
  print("IMU SUMMARY:", result["recording"])
  print("Readiness mode:", result["readiness_mode"])
  print(
    "Probe:",
    result["probe_result"],
    "| rows:",
    result["rows"],
    "| seconds:",
    round(result["duration_s"], 3),
  )
  print(
    "Missing/duplicate/nonfinite:",
    result["missing_packets"],
    result["duplicate_counters"],
    result["nonfinite_field_samples"],
  )
  print(
    "Recorder drops:",
    result["capture_dropped"],
    "| interrupted:",
    result["interrupted"],
  )
  for name, v in result["fields"].items():
    print(
      name,
      "Hz:",
      round(v["hz"], 2) if v["hz"] else None,
      "| gap ms p50/p95/p99/max:",
      "/".join(
        f"{x:.3f}" if x is not None else "NA" for x in v["host_gap_ms"].values()
      ),
      "| >=",
      v["threshold_ms"],
      "ms:",
      v["gaps_ge_threshold"],
    )
  print("CPU % of one core:", result["cpu_mean_percent_one_core"])
  if "trace" in result:
    print("Trace events/drops:", result["trace"]["events"], result["trace"]["dropped"])
  for endpoint, v in result.get("usb", {}).get("endpoints", {}).items():
    print(
      "USB",
      endpoint,
      "max gap:",
      v["payload_gap_ms"]["max"],
      "| >=5ms:",
      v["gaps_ge5ms"],
      "| full512:",
      v["full512_completions"],
      "| coverage:",
      v["coverage_relative_s"],
    )
  print("Warnings:", "; ".join(result["warnings"]) or "none")
  print("Small report:", report)
  print("Offline analysis only; host arrival gaps do not measure absolute latency.")


if __name__ == "__main__":
  main()
