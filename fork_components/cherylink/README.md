# cherylink (fork component)

CheryLink features running on the WiCAN Pro, ported from the CheryLink app
(github.com/ktshikotshi/chery-link: DriveRecorder, EnergyTracker,
EfficiencyProfile, ChargeCalibration) for the Chery Q (QQ3 EV).

- **Drive history**: start/end, km (odometer when it agrees), kWh net,
  regen kWh, Wh/km, SOC from/to, max speed, avg outside temp, A/C
  compressor kWh.
- **Charge history**: AC/DC, kWh into the pack (FD0A counter steps x pack V
  when the profile reads it, else V x I integrated, else SOC rise x
  41.278 kWh), SOC from/to, peak and average kW, duration.
- **Efficiency**: 0.1 km Wh/km bars for the last 10 km, 5 km average, 50 km
  average (the range basis: km not driven yet count at 140 Wh/km), Wh/km
  by speed band and by outside temperature band.
- **Battery**: SOC (dash + BMS), SOH, V/A/kW, cell max/min/spread, pack and
  module temps, coolant in/out, 12 V, cycle counters (q-full profile).
- **Location** (GPS add-on, `gps_latitude`/`gps_longitude`): start and end
  of every drive, a route point every 100 m (spacing doubles past 1000
  points), where every charge happened. Routes live in
  `/data/cherylink/routes/<drive start>.bin` (int32 lat/lon x1e6, last 300
  drives). The page draws them offline and links to Google Maps; "Home" is
  the place with the most charges.
- **RANGE** (est. km) and **CL_WH_KM** are published back into autopid, so
  ABRP gets `est_battery_range` and HA/MQTT see both.

## How it works

`main_glue.c` chains autopid's value sink: every accepted sample goes to
data_logger AND to `cherylink_autopid_sink` (a queue push). The
`cherylink` task (PSRAM stack) feeds `cl_engine.c` (pure C, host-tested).
It never sends anything on the bus: no extra polling, no new risk. Which
values exist depends on the vehicle profile: `vehicle_profiles/chery/q.json`
covers drives, charges and efficiency; `q-full.json` adds cells, coolant,
cycles, the charge counter and DC detection.

Storage: `/data/cherylink/` on the internal LittleFS (every access through
flash_jobs). `drives.jsonl` / `charges.jsonl` (one JSON object per line,
rotated to `*.old.jsonl` at 128 KB, ~500 records each), `state.bin` (open
drive/charge, efficiency windows, totals) saved after every record, every
10 min while data flows, and before sleep. Download or delete them on the
stock Files page.

Samples are ignored until the clock is valid (`rtc_manager_time_valid`).

## Page

`web/cherylink.html` is one self-contained file (the WiCAN AP has no
internet, ~24 KB in flash). It copies the stock web UI's colour tokens and
dark mode (web_ui_v2 index.html) and follows its saved theme choice
(`localStorage` key `wican-theme`), so it reads as part of the WiCAN UI.

## Surfaces

| Route | What |
|---|---|
| `GET /cherylink` | the page (Now, Battery, Efficiency, Drives, Charges) |
| `GET /api/cherylink` | live values with age, range, efficiency, open drive/charge, totals |
| `GET /api/cherylink/drives?limit=N` | finished drives, newest first |
| `GET /api/cherylink/charges?limit=N` | finished charges, newest first |
| `GET /api/cherylink/route?start=<epoch>` | a drive's route `[[lat,lon],...]` (the open drive's too) |
| Bluetooth FFF7/FFF8 | live values + summary + finished drives/charges pushed to the CheryLink app; ABRP handover (phone near = phone sends, phone gone = WiCAN sends over LTE). See [BLE_PROTOCOL.md](BLE_PROTOCOL.md) |

## Host test

```
cd fork_components/cherylink/host_test
cc -Wall -Wextra -I../include ../cl_engine.c test_engine.c -lm -o t && ./t
cc -Wall -Wextra -I../include ../cl_ble_core.c test_ble.c -lm -o tb && ./tb
```
