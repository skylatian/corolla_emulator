#!/usr/bin/env python3
"""Run the same pulse trains through the firmware's speed code (built by build_harness.py)
and through the Python model in vss_sim.py, and report the largest difference.

usage: python3 parity.py [route.json]   (route.json from extract_route.py; optional)
"""
import json, os, subprocess, sys
import vss_sim as S
from build_harness import build

HARNESS = build()

def via_c(pulses, t_start, t_end, dt=1/83.0):
  ev = [(round(p * 1e6) + 10**9, 'p') for p in pulses]
  t = t_start
  while t <= t_end:
    ev.append((round(t * 1e6) + 10**9, 'q')); t += dt
  ev.sort(key=lambda e: (e[0], 0 if e[1] == 'p' else 1))
  out = subprocess.run([HARNESS], input="\n".join(f"{k} {u}" for u, k in ev), capture_output=True, text=True).stdout
  return [((int(a) - 10**9) / 1e6, float(b)) for a, b in (l.split() for l in out.splitlines())]

worst = 0
cases = []
for v0 in (15, 30, 50):
  for dec in (0.5, 1.5, 3.0):
    pulses, t_stop, t_end = S.synth_stop(v0, dec)
    cases.append((f"synth {v0}kph {dec}m/s2", pulses, pulses[0], t_end, t_stop))
if len(sys.argv) > 1:
  d = json.load(open(sys.argv[1]))
  rp = []
  for (ta, _), (tb, n) in zip(d["d502"], d["d502"][1:]):
    rp += [ta + (k + 0.5) * (tb - ta) / n for k in range(n)]
  cases.append((f"route {os.path.basename(sys.argv[1])}", rp, d["d502"][0][0], d["d502"][-1][0], None))
for name, pulses, t0, t1, t_stop in cases:
  c = via_c(pulses, t0, t1)
  py = S.run(S.New, pulses, t1, t_start=t0)
  n = min(len(c), len(py))
  diff = max(abs(c[i][1] - py[i][1]) for i in range(n))
  worst = max(worst, diff)
  extra = ""
  if t_stop is not None:
    extra = f"  C settles to 0 at {S.first_settled_zero(c, 0) - t_stop:+.2f} s vs stop"
  print(f"{name:28s} samples {n:6d}  max |C - Python| = {diff:.2f} kph{extra}")
print("worst", round(worst, 2))
