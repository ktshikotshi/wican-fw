# CheryLink phone feed: the WiCAN side (protocol 1, Bluetooth + Wi-Fi)

WiCAN polls the car with autopid (the Chery Q profile). The CheryLink app
connects over Bluetooth and **receives** what WiCAN reads. The app puts
nothing on the bus and needs no ELM adapter. The same link decides who
sends to ABRP:

- **Phone in the car** (connected and sending heartbeats with `"abrp":true`):
  the app sends to ABRP with its extras (CarLinko tyres, A/C set
  temperature) and WiCAN's own ABRP destination is **held**.
- **Phone gone** (no hello or heartbeat for 60 s): WiCAN sends to ABRP
  itself over its uplink (the LTE stick), using its ABRP destination
  (configured on the WiCAN web page with the user's ABRP token).

ABRP never gets two senders at once.

## Transport

The WiCAN Pro GATT service `0xFFF0` (see the stock
`ble_manager/BLE_API.md` for advertising, pairing, MTU and the Wi-Fi
handover). The CheryLink channel adds one pair of characteristics:

| Characteristic | UUID (16-bit on the BT base) | Properties | Direction |
|---|---|---|---|
| CheryLink OUT | `0xFFF7` (`0000FFF7-0000-1000-8000-00805F9B34FB`) | indicate | WiCAN to app |
| CheryLink IN  | `0xFFF8` (`0000FFF8-0000-1000-8000-00805F9B34FB`) | write, write-without-response | app to WiCAN |

- The link must be **paired** (encrypted + authenticated). Subscribing to
  FFF7 on a plain link gets "Insufficient Authentication", and the phone
  pairs right there. Pairing needs the WiCAN pairing window open the first
  time (WiCAN web page, Bluetooth, or its button).
- Subscribe to FFF7 (iOS `setNotifyValue(true)`, Android
  `ENABLE_INDICATION_VALUE`). WiCAN starts sending once it sees the
  subscription.
- Android: `requestMtu(517)` right after connecting. Frames are split over
  as many PDUs as needed, so do not rely on the MTU. Reassemble bytes and
  split on `\n`.
- While a phone is connected, WiCAN suspends its own Wi-Fi (stock
  behaviour). Its HTTP API stays reachable through the stock HTTP tunnel
  (FFF3/FFF4, `ble_http/BLE_HTTP_PROTOCOL.md`). Use it for history:
  `GET /api/cherylink/drives`, `/api/cherylink/charges`,
  `/api/cherylink/route?start=`.
- One central at a time.

## Wi-Fi transport (same protocol)

The same feed runs over **TCP port 35001** on every WiCAN interface (its
own access point `192.168.0.10`, or its address on your Wi-Fi). Connect,
then read and write exactly the bytes described below (newline-delimited
JSON both ways, the same `hello` / `v` / `s` / `trip` / `charge` frames
and the same `hello` / `hb` / `bye` messages). The `hello` frame says
`"link":"wifi"` (or `"ble"`).

- One phone at a time: a new connection replaces the previous one.
- Nothing to subscribe: frames start right after the connect.
- The ELM327 port (35000) stays as before, for scan programs and the
  module map. A phone on the feed does not need it. While an app talks
  ELM on 35000, WiCAN's own polling pauses (stock), so the feed goes
  quiet. Use one or the other.
- No pairing on this port. Anyone who can reach the WiCAN on Wi-Fi can
  read the feed (as with the ELM port), so keep the WiCAN access point's
  Wi-Fi password set. WiCAN's LTE uplink takes no inbound connections.
- Bluetooth and Wi-Fi can run at the same time. ABRP is held while
  either phone link says `abrp:true`.

## Security

- Only a **paired** phone gets anything: every characteristic needs an
  encrypted + authenticated link (LE Secure Connections with a 6-digit
  passkey, MITM-protected). The passkey is the WiCAN `ble` setting
  `passkey`. **Change it from the default 123456.**
- New pairings are only accepted while the **pairing window** is open.
  The stock default `pairing_at_boot` = true opens it at every boot, and
  WiCAN reboots on every wake, so the window opens at every car start.
  **Turn `pairing_at_boot` off** once the phone is paired. To pair
  another phone later, use the /cherylink page: "Allow a new phone to
  pair (2 min)" (`POST /api/cherylink/pair`), then pair from the app.
- At most 3 bonded phones. A 4th pairing evicts the oldest bond.
- Turn on the WiCAN admin password (`http_server_manager.auth_enabled`)
  so nobody on the WiCAN Wi-Fi can open the pairing window or change the
  settings.

## Framing

UTF-8 JSON, **one object per line**, each line ends with `\n`. Every
object has `"t"` (its type). Ignore unknown types and unknown fields
(newer firmware may add them).

## WiCAN to app (FFF7)

### `hello`: once per subscription

```json
{"t":"hello","proto":1,"fw":"6.0.0-beta.7-fork","pack_kwh":41.278,"clock_valid":true,"abrp":"wican"}
```

`abrp` is who sends to ABRP right now: `"phone"` or `"wican"`.

### `v`: live values, every second (only what changed)

```json
{"t":"v","ts":1793734500,"v":{"SOC":81.25,"HV_W":-3.5,"SPEED":62,"gps_latitude":-26.123456}}
```

- Keys are the **autopid parameter names** of the WiCAN profile
  (`vehicle_profiles/chery/q.json`, `q-full.json`), plus the GPS add-on's
  `gps_latitude`, `gps_longitude`, `gps_altitude`, `gps_speed`,
  `gps_heading`, `gps_satellites`, plus cherylink's own `RANGE` (km) and
  `CL_WH_KM`.
- The values are numbers in the profile's units (`SOC` %, `HV_V` V, `HV_A`
  A, `HV_W` kW (+ = out of the pack), `SPEED` km/h, ...). `null` = not a
  number.
- `ts` = WiCAN's clock (epoch s) when the frame left. A value is as fresh
  as its frame: values that did not change are not repeated. Right after
  the subscription every known value is sent once (it may take a few
  frames).
- `SOC` is already the dash SOC (`floor((BMS-2)/0.98)` in the profile).
  Do not map it again.

### `s`: summary, every 10 s

```json
{"t":"s","ts":1793734510,"abrp":"phone","s":{"loaded":true,"est_range_km":218,"range_wh_km":141,"avg_wh_km_5":136,"instant_wh_km":null,"trip":null,"charge":{...},"totals":{"km":1234.5,"kwh":171.20,"charged_kwh":180.40,"drives":96,"charges":21}}}
```

`trip` / `charge` = the open drive or charge in the same shape as the
history lines (see `README.md`), or `null`.

### `trip` / `charge`: a drive or charge just finished

```json
{"t":"trip","r":{"start":1793734500,"end":1793736300,"km":24.31,"kwh":3.402,...}}
{"t":"charge","r":{"start":...,"dc":false,"kwh":12.5,...}}
```

`r` is the same JSON as one line of `/api/cherylink/drives` or
`/charges`. A record is sent once. One missed while no phone was
connected stays in the history API.

## App to WiCAN (FFF8)

One JSON object per line, `\n` terminated, at most 255 bytes.

| Message | When | Effect |
|---|---|---|
| `{"t":"hello","app":"ios","proto":1,"abrp":true}` | after subscribing | `abrp:true` = "I send to ABRP": WiCAN holds its own ABRP sender |
| `{"t":"hb","abrp":true}` | every 10-20 s while connected | keeps the hold, which lapses **60 s** after the last hello/hb |
| `{"t":"hb","abrp":false}` | the app's ABRP is off | WiCAN sends to ABRP |
| `{"t":"bye"}` | the app closes the link on purpose | WiCAN sends to ABRP at once |

The app should send `abrp:true` only when its own ABRP sending is on and
it has an uplink. If it loses the internet, send `abrp:false` so WiCAN
takes over (WiCAN has LTE).

## On the WiCAN side

- `cl_ble.c` is the glue (task `cl_ble`), `cl_ble_core.c` the logic
  (host test `host_test/test_ble.c`).
- The ABRP hold is `data_destinations_set_abrp_hold()`
  (`patches/meatpi-components/0004-data-destinations-abrp-hold.patch`).
  While held, due ABRP pushes are skipped without a request. The log says
  `abrp held (another sender owns it)` / `abrp resumed`.
- Bluetooth must be enabled in WiCAN settings (stock `ble` settings).
