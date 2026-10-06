"""Offline CSV plots from imu_probe --record. No device or DDS access."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("csv", type=Path)
p.add_argument("--output", type=Path)
a = p.parse_args()
out = a.output or a.csv.with_suffix("")
out.mkdir(parents=True, exist_ok=True)
r = np.atleast_1d(np.genfromtxt(a.csv, delimiter=",", names=True))
if len(r) < 2:
  raise ValueError("Recording needs at least two packets")
t = r["host_monotonic_s"] - r["host_monotonic_s"][0]
mask = r["field_mask"].astype(int)
if not np.all(np.diff(t) >= 0):
  raise ValueError("Nonmonotonic host timestamps")
plt.rcParams.update({"axes.grid": True, "grid.alpha": 0.2})
for bit, names, title, units in [
  (1, ["qw", "qx", "qy", "qz"], "orientation", "degrees"),
  (2, ["gx", "gy", "gz"], "gyro", "rad/s"),
  (4, ["ax", "ay", "az"], "acceleration", "m/s² (gravity included)"),
]:
  take = (mask & bit) != 0
  v = np.column_stack([r[n][take] for n in names])
  if not len(v):
    continue
  if bit == 1:
    norm = np.linalg.norm(v, axis=1)
    if not np.all(np.isfinite(norm) & (norm > 0.5) & (norm < 1.5)):
      raise ValueError(
        "Invalid quaternion present; inspect recording before plotting attitude"
      )
    w, x, y, z = (v / norm[:, None]).T
    v = np.rad2deg(
      np.column_stack(
        [
          np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
          np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
          np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
        ]
      )
    )
  fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8), layout="constrained")
  for j, ax in enumerate(axes):
    ax.plot(t[take], v[:, j], linewidth=0.65)
    ax.set_ylabel(
      (["roll", "pitch", "yaw"][j] if bit == 1 else "xyz"[j]) + "\n" + units
    )
    ax.ticklabel_format(axis="y", useOffset=False)
  axes[-1].set_xlabel("Host callback time since first recorded packet [s]")
  fig.suptitle(
    f"{a.csv.name}: {title}\nFresh field samples only; no smoothing; no automatic body-frame mounting correction"
  )
  fig.savefig(out / (title + ".png"), dpi=140)
  plt.close(fig)

fig, axes = plt.subplots(3, 1, figsize=(12, 8), layout="constrained")
for bit, name in [(1, "orientation"), (2, "gyro"), (4, "acceleration")]:
  take = (mask & bit) != 0
  tt = t[take]
  if len(tt) < 2:
    continue
  gaps = np.diff(tt) * 1000
  axes[0].plot(tt[1:], gaps, ".", markersize=2, label=name)
  ticks = r["sample_time_fine_ticks"][take]
  valid = (ticks[1:] >= 0) & (ticks[:-1] >= 0)
  # Device 32-bit timestamp rollover; display gaps, not cross-clock latency.
  device_gaps = np.mod(np.diff(ticks), 2**32) * 0.1
  axes[1].plot(tt[1:][valid], device_gaps[valid], ".", markersize=2, label=name)
  xs = np.sort(gaps)
  axes[2].plot(xs, np.arange(1, len(xs) + 1) / len(xs) * 100, label=name)
  print(
    name,
    "samples",
    len(tt),
    "host gap ms p50/p95/p99/max",
    np.percentile(gaps, [50, 95, 99, 100]),
  )
axes[0].set_ylabel("Host update gap [ms]")
axes[1].set_ylabel("Device timestamp gap [ms]")
axes[0].set_xlabel("Host time [s]")
axes[1].set_xlabel("Host time [s]")
axes[2].set_xlabel("Host update gap [ms]")
axes[2].set_ylabel("Cumulative %")
for ax in axes:
  ax.legend()
fig.suptitle(
  "Delivery timing — clock domains are separate; this does not measure absolute latency"
)
fig.savefig(out / "timing.png", dpi=140)
plt.close(fig)
print("Plots saved:", out)
