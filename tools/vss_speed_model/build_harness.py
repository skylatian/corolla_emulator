#!/usr/bin/env python3
"""Compile the VSS sketch's own speed code on the host, with a fake clock.

Copies the `#define VSS_* / EMA_K` lines, `vssISR()` + `pps_to_kmh100()`, and the loop's
speed block (from "--- VSS speed calculation" to "--- Gear selection") out of
vss_speed/vss_speed.ino into vss_harness.cpp, then builds ./vss_harness.

The harness reads stdin lines `p <us>` (a pulse at that micros() time) or `q <us>`
(run the speed block at that time and print "<us> <kph>").
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
INO = os.path.join(HERE, "..", "..", "vss_speed", "vss_speed.ino")
SEP = "// ============================================================================"


def build(out_dir=HERE):
  ino = open(INO).read()
  defs = "\n".join(l for l in ino.splitlines() if re.match(r"#define (VSS_|EMA_K)", l))
  isr_start = ino.index("void vssISR()")
  isr = ino[isr_start:ino.index(SEP, ino.index("uint16_t pps_to_kmh100"))]
  block = ino[ino.index("  // --- VSS speed calculation"):ino.index("  // --- Gear selection")]
  src = f"""#include <cstdio>
#include <cstdint>
{defs}
static unsigned long g_now = 0;
unsigned long micros() {{ return g_now; }}
void noInterrupts() {{}}
void interrupts() {{}}
volatile unsigned long vssPulseTimes[VSS_RING_SIZE];
volatile uint8_t vssRingIdx = 0, vssPulseStored = 0, vssPulseSeq = 0;
volatile uint16_t vssPulseCount = 0;
uint16_t speedKmh100 = 0, rawSpeedKmh100 = 0;
uint32_t smoothedX256 = 0;
uint8_t lastPulseSeq = 0;
unsigned long avgPeriodUs = 0;
{isr}
void speed_step() {{
{block}
}}
int main() {{
  char c; unsigned long t;
  while (scanf(" %c %lu", &c, &t) == 2) {{
    g_now = t;
    if (c == 'p') vssISR();
    else {{ speed_step(); printf("%lu %.2f\\n", t, speedKmh100 / 100.0); }}
  }}
}}
"""
  cpp = os.path.join(out_dir, "vss_harness.cpp")
  exe = os.path.join(out_dir, "vss_harness")
  open(cpp, "w").write(src)
  subprocess.run(["c++", "-std=c++11", "-O1", "-o", exe, cpp], check=True)
  return exe


if __name__ == "__main__":
  print(build(sys.argv[1] if len(sys.argv) > 1 else HERE))
