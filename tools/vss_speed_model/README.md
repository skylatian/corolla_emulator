# VSS speed model

Offline tools for checking changes to the speed logic in `vss_speed/vss_speed.ino`
before flashing. Used for VSS `100926a` (2026-10-09); write-up in the parent docs repo,
`project_docs/phase3-emulator-2026-09-27-ecu-diagnostics.md` §9.

| File | What it does |
|------|--------------|
| `vss_sim.py` | Python model of the speed logic (`Old` = 092826b, `New` = 100926a). Synthetic stops, and replay of a route's pulse counts. |
| `build_harness.py` | Copies the sketch's own speed code into `vss_harness.cpp` and compiles it for the Mac with a fake `micros()`. |
| `parity.py` | Runs the same pulses through the compiled sketch code and the `New` model; reports the largest difference. |
| `extract_route.py` | Pulls 0xAA speed, 0x502 pulse counts and GPS out of a route into JSON (needs openpilot, run via `./dev python`). |

Typical use after editing the speed code:

```bash
cd StarPilot && ./dev python ../corolla_emulator/tools/vss_speed_model/extract_route.py ../logs/log_dump <route> /tmp/route.json
cd ../corolla_emulator/tools/vss_speed_model
python3 vss_sim.py /tmp/route.json   # update the New model first if the logic changed
python3 parity.py /tmp/route.json    # C vs model; expect a few hundredths of a kph
```

Limits: the route replay spreads each 0.5 s window's pulses evenly, so it approximates
real pulse timing. Synthetic stops use a proportional pulse rate (about 0.75 pulses per
second per kph).
`vss_harness` and `vss_harness.cpp` are build outputs (ignored by git).
