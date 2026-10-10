#!/usr/bin/env python3
#!/usr/bin/env python3
"""Offline model of the VSS speed logic: `Old` = firmware 092826b, `New` = 100926a.

Both see pulse timestamps (as the ISR does) and are evaluated at the 0xAA send rate
(83 Hz). The C is checked against `New` by parity.py; if the sketch's speed logic
changes, update `New` (and keep the old one as `Old`) before comparing.

usage: python3 vss_sim.py [route.json]
  - synthetic constant-decel stops: how long after the true stop each reads 0
  - with route.json (extract_route.py): pulse trains rebuilt from the 0x502 counts
    (spread evenly across each 0.5 s window), speed from counts vs logged 0xAA vs GPS,
    and stop lag after the last pulse
"""
import json, math, sys

RING = 8
EMA_K = 3
STALE_S = 2.0
WINDOW_S = 0.25
LOW_PPS = 2.0  # below this, proposed fit is a line through zero
STOP_S = 0.5   # proposed: no pulse for this long -> report standstill (ceiling is ~1.1 kph by then)


def fit_old(pps):
  kph = 1.45 * pps - 1.81 if pps >= 1.0 else pps
  return max(kph, 0.0)


def fit_new(pps):
  if pps >= LOW_PPS:
    return 1.45 * pps - 1.81
  return pps * (1.45 * LOW_PPS - 1.81) / LOW_PPS


class Old:
  """092826b: average of the last 8 periods, EMA per pulse, 2 s stale -> 0."""
  def __init__(self):
    self.periods = []
    self.last = None
    self.smooth = 0.0
    self.seen = 0

  def pulse(self, t):
    if self.last is not None:
      self.periods = (self.periods + [t - self.last])[-RING:]
    self.last = t

  def speed(self, now, n_pulses):
    if self.last is None or now - self.last > STALE_S:
      self.smooth = 0.0
      return 0.0
    raw = fit_old(1.0 / (sum(self.periods) / len(self.periods))) if self.periods else 0.0
    if n_pulses != self.seen:
      self.seen = n_pulses
      self.smooth += (raw - self.smooth) / EMA_K
    return self.smooth


class New:
  """Proposed: time-window average (>=2 pulses, <=8), EMA per pulse, ceiling from time
  since the last pulse, 2 s stale -> 0."""
  def __init__(self):
    self.times = []
    self.smooth = 0.0
    self.seen = 0
    self.avg_period = None

  def pulse(self, t):
    self.times = (self.times + [t])[-RING:]

  def speed(self, now, n_pulses):
    if not self.times or now - self.times[-1] > STALE_S:
      self.smooth = 0.0
      return 0.0
    last = self.times[-1]
    if len(self.times) >= 2:
      k = 2
      while k < len(self.times) and last - self.times[-k - 1] <= WINDOW_S:
        k += 1
      first = self.times[-k]
      self.avg_period = (last - first) / (k - 1)
      raw = fit_new(1.0 / self.avg_period)
    else:
      raw = 0.0
    if n_pulses != self.seen:
      self.seen = n_pulses
      self.smooth += (raw - self.smooth) / EMA_K
    out = self.smooth
    elapsed = now - last
    if elapsed > STOP_S:
      return 0.0  # EMA keeps its state; the next pulse after a real stop restarts it via stale
    if self.avg_period is not None and elapsed > self.avg_period:
      out = min(out, fit_new(1.0 / elapsed))
    return out


def run(algo_cls, pulses, t_end, dt=1 / 83.0, t_start=None):
  a = algo_cls()
  t = pulses[0] - 1.0 if t_start is None else t_start
  i = 0
  out = []
  while t <= t_end:
    while i < len(pulses) and pulses[i] <= t:
      a.pulse(pulses[i])
      i += 1
    out.append((t, a.speed(t, i)))
    t += dt
  return out


def first_settled_zero(series, after):
  """Time from which the output stays exactly 0 for the rest of the series."""
  z = None
  for t, v in series:
    if t < after:
      continue
    if v <= 0.0:
      z = t if z is None else z
    else:
      z = None
  return z


# ---------------- synthetic stops ----------------
PPS_PER_KPH = 1 / 1.33  # proportional pulse rate for synthetic profiles (20 kph ~ 15 pps, like the fit)


def synth_stop(v0_kph, decel_ms2, hold_s=4.0):
  """Constant decel from v0 to stop, then stand still. Returns pulses, true-speed fn, stop time."""
  v0 = v0_kph / 3.6
  t_stop = v0 / decel_ms2
  pulses, phase, t, dt = [], 0.0, 0.0, 1e-4
  while t < t_stop:
    v_kph = (v0 - decel_ms2 * t) * 3.6
    phase += v_kph * PPS_PER_KPH * dt
    if phase >= 1.0:
      phase -= 1.0
      pulses.append(t)
    t += dt
  pre = [-(k + 1) * (1 / (v0_kph * PPS_PER_KPH)) for k in range(40)][::-1]  # steady cruise before
  return pre + pulses, t_stop, t_stop + hold_s


def synth_report():
  print("Synthetic stops (constant decel to standstill). 'lag' = output settles at 0 after the true stop.")
  print(f"{'v0 kph':>7} {'decel':>6} | {'old lag':>8} {'new lag':>8} | {'old err @-1s':>12} {'new err @-1s':>12}")
  for v0 in (15, 30, 50):
    for dec in (0.5, 1.5, 3.0):
      pulses, t_stop, t_end = synth_stop(v0, dec)
      true = lambda t: max(v0 - dec * 3.6 * t, 0.0) if t >= 0 else v0
      o = run(Old, pulses, t_end, t_start=pulses[0])
      n = run(New, pulses, t_end, t_start=pulses[0])
      lo = first_settled_zero(o, 0) - t_stop
      ln = first_settled_zero(n, 0) - t_stop
      tm = t_stop - 1.0
      eo = next(v for t, v in o if t >= tm) - true(tm)
      en = next(v for t, v in n if t >= tm) - true(tm)
      print(f"{v0:7d} {dec:6.1f} | {lo:8.2f} {ln:8.2f} | {eo:12.2f} {en:12.2f}")


# ---------------- log replay ----------------
def log_report(path):
  d = json.load(open(path))
  aa, d502 = d["aa"], d["d502"]
  # Rebuild pulse times: spread each 0x502 window's count evenly across the window.
  pulses = []
  for (ta, _), (tb, n) in zip(d502, d502[1:]):
    for k in range(n):
      pulses.append(ta + (k + 0.5) * (tb - ta) / n)
  if not pulses:
    print("no pulses"); return
  t0, t1 = d502[0][0], d502[-1][0]
  o = run(Old, pulses, t1, t_start=t0)
  n = run(New, pulses, t1, t_start=t0)

  # Pulse count vs reported speed, per 2 s: speed computed straight from the 0x502 counts,
  # logged 0xAA (max and median: the logged firmware 092726a still had the dips, which only
  # pull values down), both sims, and GPS.
  import bisect, statistics
  gps = d["gps"]
  at = [a[0] for a in aa]; ot = [x[0] for x in o]; gt = [g[0] for g in gps]; dt5 = [x[0] for x in d502]
  rows = []
  t = t0
  while t + 2 < t1:
    seg = lambda s, ts: [v for (tt, v) in s[bisect.bisect_left(ts, t):bisect.bisect_left(ts, t + 2)]]
    la, so, sn, sg = seg(aa, at), seg(o, ot), seg(n, ot), seg(gps, gt)
    w = d502[bisect.bisect_left(dt5, t):bisect.bisect_left(dt5, t + 2)]
    if la and so and sn and len(w) >= 2:
      span = w[-1][0] - w[0][0]
      cnt = sum(c for _, c in w[1:])
      pps = cnt / span if span > 0 else 0
      rows.append((t, fit_old(pps), max(la), statistics.median(la), statistics.median(so),
                   statistics.median(sn), statistics.median(sg) if sg else float("nan")))
    t += 2
  mv = [r for r in rows if r[1] > 3]
  print(f"\nLog replay ({len(pulses)} rebuilt pulses), {len(mv)} moving 2 s windows")
  mean = lambda xs: statistics.mean(xs) if xs else float("nan")
  print(f"  mean |count-derived - logged max|    = {mean([abs(r[1]-r[2]) for r in mv]):.2f} kph")
  print(f"  mean |count-derived - old sim|       = {mean([abs(r[1]-r[4]) for r in mv]):.2f} kph   (sanity: sim matches the counts)")
  print(f"  mean |old sim - new sim|             = {mean([abs(r[4]-r[5]) for r in mv]):.2f} kph   (new logic's effect while moving)")
  g = [r for r in mv if not math.isnan(r[6]) and r[6] > 3]
  print(f"  mean |count-derived - GPS| ({len(g)} w/ fix) = {mean([abs(r[1]-r[6]) for r in g]):.2f} kph, "
        f"mean (count-derived - GPS) = {mean([r[1]-r[6] for r in g]):+.2f} kph")
  print("   t     counts  logged max/med   old sim  new sim   GPS")
  for r in rows[::10]:
    print(f"  {r[0]:5.0f}  {r[1]:6.1f}   {r[2]:6.1f} {r[3]:6.1f}   {r[4]:6.1f}  {r[5]:6.1f}  {r[6]:5.1f}")

  # stops: last rebuilt pulse followed by >= 2 s of none
  print("  stops: last pulse -> output settled at 0")
  for a, b in zip(pulses, pulses[1:] + [t1 + 10]):
    if b - a > STALE_S + 0.5:
      zo = first_settled_zero([x for x in o if x[0] <= min(b, t1)], a)
      zn = first_settled_zero([x for x in n if x[0] <= min(b, t1)], a)
      la = [x for x in aa if a - 0.5 <= x[0] <= min(b, t1)]
      zl = first_settled_zero(la, a) if la else None
      f = lambda z: "  n/a" if z is None else f"{z - a:5.2f}"
      print(f"    t={a:7.2f}s  logged {f(zl)} s   old sim {f(zo)} s   new sim {f(zn)} s")


if __name__ == "__main__":
  synth_report()
  if len(sys.argv) > 1:
    log_report(sys.argv[1])
