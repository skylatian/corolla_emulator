"""Extract VSS data from a route for vss_sim.py / parity.py, and list its stops.

Needs openpilot's LogReader, so run it through StarPilot's host runtime:
  cd StarPilot && ./dev python ../corolla_emulator/tools/vss_speed_model/extract_route.py \
      ../logs/log_dump 0000037d /tmp/route.json

Writes {"aa": [[t, kph]...], "d502": [[t, pulses]...], "gps": [[t, kph]...]} (t in s from
the first message) and prints, per stop, when the logged 0xAA reached 0 after the last
0x502 window that had pulses. Needs VSS firmware with the 0x502 frame (092726a+).
"""
import os, sys, bisect, json
from openpilot.tools.lib.logreader import LogReader
root, pre = sys.argv[1], sys.argv[2]
segs = sorted((d for d in os.listdir(root) if d.startswith(pre) and os.path.exists(os.path.join(root, d, "rlog.zst"))),
              key=lambda d: int(d.rsplit("--", 1)[1]))
aa, d502, gps = [], [], []
t0 = None
for d in segs:
  for m in LogReader(os.path.join(root, d, "rlog.zst"), sort_by_time=True):
    t = m.logMonoTime; t0 = t0 or t; w = m.which()
    if w == "can":
      for c in m.can:
        if c.src != 0: continue
        b = bytes(c.dat)
        if c.address == 0xAA: aa.append(((t - t0) / 1e9, (((b[0] << 8) | b[1]) - 6767) / 100.0))
        elif c.address == 0x502: d502.append(((t - t0) / 1e9, b[5]))
    elif w == "gpsLocationExternal" and m.gpsLocationExternal.hasFix:
      gps.append(((t - t0) / 1e9, m.gpsLocationExternal.speed * 3.6))
json.dump({"aa": aa, "d502": d502, "gps": gps}, open(sys.argv[3], "w"))
at = [a[0] for a in aa]
print(f"0xAA {len(aa)}  0x502 {len(d502)}  gps {len(gps)}  span {aa[-1][0]:.0f}s")
# stops: pulse window >0 followed by >=4 windows (2 s) of 0
print("stop#  last_pulse_window_end  aa_first_zero  lag_after_window_end  pulses(last 4 windows)  gps_at_window_end")
n = 0
for i in range(4, len(d502) - 4):
  if d502[i][1] > 0 and all(d502[i + k][1] == 0 for k in range(1, 5)):
    tw = d502[i][0]
    j = bisect.bisect_left(at, tw - 0.5)
    z = next((a[0] for a in aa[j:] if a[1] <= 0.0 and a[0] > tw - 0.5), None)
    gi = bisect.bisect_left([g[0] for g in gps], tw)
    g = gps[gi][1] if gi < len(gps) and abs(gps[gi][0] - tw) < 1.5 else float("nan")
    n += 1
    print(f"{n:5d}  {tw:8.2f}  {z if z is None else round(z,2)!s:>8}  {'' if z is None else round(z - tw, 2)!s:>6}  {[d502[i - k][1] for k in (3,2,1,0)]}  {g:5.1f}")
