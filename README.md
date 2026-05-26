# esphome-meshcore

An [ESPHome](https://esphome.io) external component that embeds the
[MeshCore](https://github.com/meshcore-dev/MeshCore) LoRa mesh library
into an ESP32-based ESPHome node. Lets you build a Home Assistant
connected mesh endpoint without writing PlatformIO firmware from
scratch.

> Status: **builds end-to-end** against ESPHome 2026.5 + arduino-esp32
> 3.3.8 + MeshCore 1.10. Validated on Heltec WiFi LoRa 32 V3 (SX1262)
> and TTGO T-Beam (SX1276). Identity persists across reboots, RTC
> learns time from the mesh, channel messaging is wire-compatible with
> upstream `BaseChatMesh`. See [Roadmap](#roadmap) for what's still
> open.

## Layout

```
esphome-meshcore/
├── components/
│   └── meshcore/
│       ├── __init__.py          # hub schema + lib_deps + radio defines
│       ├── meshcore.h           # MeshCoreComponent + EsphomeMesh + sub-sensors
│       ├── meshcore.cpp         # setup() / loop() / send_text_message() / mesh callbacks
│       ├── automation.h         # SendTextMessageAction
│       ├── sensor/
│       │   └── __init__.py      # rssi / snr / battery_voltage sensors
│       └── text_sensor/
│           └── __init__.py      # last_message text_sensor
├── libs/
│   └── ed25519/                 # vendored orlp/ed25519 (MeshCore needs ed_25519.h)
│       ├── library.json
│       └── src/                 # add_scalar.c, fe.c, ge.c, key_exchange.c, ...
├── example.yaml                 # minimal end-to-end YAML
└── README.md
```

## Quick start

> **Requires `framework: arduino`** on the ESP32. MeshCore and RadioLib
> pull in `<SPI.h>`, `<Arduino.h>`, and friends that don't exist under
> the esp-idf framework. The component fails config validation with a
> clear message if you try `framework: esp-idf`.

1. Drop the repo into your ESPHome YAML:

   ```yaml
   external_components:
     - source:
         type: git
         url: https://gitea.blackax.net/blackax/esphome-meshcore.git
         ref: main
         refresh: 0s        # while iterating; remove once you pin a tag
       components: [meshcore]
   ```

2. Configure the hub against your LoRa module pins. Note we own the
   SPI bus directly — do **not** also add an `spi:` block. MeshCore's
   helpers call `SPI.begin()` internally with the pins we provide.

   ```yaml
   meshcore:
     id: mesh_hub
     radio: sx1262
     sclk_pin: GPIO9
     miso_pin: GPIO11
     mosi_pin: GPIO10
     cs_pin: GPIO8
     dio1_pin: GPIO14
     reset_pin: GPIO12
     busy_pin: GPIO13
     channels:
       - name: "Public"
         key: "izOH6cXN6mrJ5e26oRXNcg=="

   text_sensor:
     - platform: meshcore
       meshcore_id: mesh_hub
       last_message:
         name: "Mesh Last Message"

   sensor:
     - platform: meshcore
       meshcore_id: mesh_hub
       rssi:
         name: "Mesh Last RSSI"
       snr:
         name: "Mesh Last SNR"
       battery_voltage:
         name: "Mesh Battery Voltage"
   ```

3. Send a message from a HA automation or button:

   ```yaml
   button:
     - platform: template
       name: "Send Mesh Message"
       on_press:
         then:
           - meshcore.send_text_message:
               id: mesh_hub
               channel: "Public"
               text: "Hello from ESPHome!"
   ```

See [`example.yaml`](./example.yaml) for a complete file.

## Configuration reference

### `meshcore:` block

| Field | Required? | Default | Notes |
|---|---|---|---|
| `id` | yes | — | ESPHome component id; how other platforms reference this hub |
| `radio` | yes | — | One of `sx1262`, `sx1276`, `sx1278` |
| `sclk_pin` | yes | — | LoRa SPI clock |
| `miso_pin` | yes | — | LoRa SPI MISO |
| `mosi_pin` | yes | — | LoRa SPI MOSI |
| `cs_pin` | yes | — | LoRa SPI chip-select |
| `reset_pin` | yes | — | LoRa reset line |
| `busy_pin` | sx126x only | — | SX126x BUSY line. Required for `radio: sx1262`, must be omitted for sx127x |
| `dio0_pin` | sx127x only | — | SX127x packet-done IRQ. Required for `sx1276`/`sx1278`, must be omitted for sx126x |
| `dio1_pin` | sx126x only | — | SX126x: required (only IRQ line). SX127x: optional; omit on boards that don't wire DIO1 (T-Beam classic, Heltec V1) — RadioLib falls back to polling |
| `frequency` | no | `869.618` MHz | Override for non-EU868 regions (US: 906–927 MHz, AU: 915–928 MHz, AS: 433 MHz). Range 137–1020 MHz |
| `bandwidth` | no | `62.5` kHz | One of `7.8 / 10.4 / 15.6 / 20.8 / 31.25 / 41.7 / 62.5 / 125 / 250 / 500` kHz |
| `spreading_factor` | no | `8` | Range 5–12. Lower = faster + shorter range |
| `coding_rate` | no | `5` | Range 5–8 (interpreted as 4/`N`) |
| `tx_power` | no | `17` dBm | Range -9 to 22. SX1276 PA_BOOST tops out around 17 with RadioLib defaults |
| `tcxo_voltage` | sx126x only | `1.6` V | Range 0.0–3.3 V. Set to 0 for boards with XOSC instead of TCXO |
| `dio2_as_rf_switch` | sx126x only | `false` | Some boards wire DIO2 to a PA T/R switch instead of a fixed pin |
| `rx_boosted_gain` | sx126x only | `false` | SX1262 high-sensitivity RX. ~5 mA more current draw |
| `node_name` | no | `esphome-mesh` | Up to 32 bytes. Becomes both the contact-list label and the chat sender prefix |
| `role` | no | `companion` | One of `companion` (RX/TX only), `repeater` (also flood-forwards traffic) |
| `private_key` | no | (auto) | Hex string: 64 bytes (prv only) or 96 bytes (prv+pub). When set, overrides the NVS cache and pins identity across reflashes. Use `!secret` |
| `battery_pin` | no | — | ADC pin for `mesh::MainBoard::getBattMilliVolts` if your board exposes a battery divider |
| `gps_latitude` | no | — | Latitude for self-advert packets (used with `gps_longitude`) |
| `gps_longitude` | no | — | Longitude for self-advert packets (used with `gps_latitude`) |
| `channels` | no | `[]` | List of `name` + `key` entries; see channel reference below |

### `channels:` entries

| Field | Required? | Notes |
|---|---|---|
| `name` | yes | Human-readable label, must match what other clients use for the same PSK |
| `key` | yes | Pre-shared key. Accepts either base64 (`izOH6cXN6mrJ5e26oRXNcg==`) or hex (`f8f5e10540e58c73170ec33d1b7194bf`). Must decode to 16 or 32 bytes |

The upstream MeshCore "Public" channel PSK is
`izOH6cXN6mrJ5e26oRXNcg==`. Use a different key for a private channel
(generate with `openssl rand 16 | base64`).

### `text_sensor:` (sub-platform)

```yaml
text_sensor:
  - platform: meshcore
    meshcore_id: <hub id>
    last_message:        # optional; only added if you list it
      name: "Mesh Last Message"
```

Publishes the parsed `"<sender>: <text>"` string from the most recent
`PAYLOAD_TYPE_GRP_TXT` packet on any configured channel.

### `sensor:` (sub-platform)

```yaml
sensor:
  - platform: meshcore
    meshcore_id: <hub id>
    rssi:                # optional
      name: "Mesh Last RSSI"
    snr:                 # optional
      name: "Mesh Last SNR"
    battery_voltage:     # optional; needs battery_pin on the hub
      name: "Mesh Battery Voltage"
```

`rssi`/`snr` reflect the most recent received packet. `battery_voltage`
samples once a minute via `mesh::MainBoard::getBattMilliVolts`.

### GPS coordinates in self-advert

Set `gps_latitude` and `gps_longitude` on the `meshcore:` block to include
GPS coordinates in your node's self-advert packet. Other nodes that receive
the advert will store the location in their routing tables, enabling
location-based routing (hops, distance estimates) and letting you see where
nodes live on a map.

```yaml
meshcore:
  id: mesh_hub
  radio: sx1262
  # ... pins ...
  gps_latitude: 37.7749
  gps_longitude: -122.4194
  channels:
    - name: "Public"
      key: "izOH6cXN6mrJ5e26oRXNcg=="
```

Both fields use ESPHome's `cv.latitude` / `cv.longitude` validators
(±90 / ±180 range). Leave them unset if the node has no GPS fix — the
advert is sent with `lat = 0, lon = 0` which upstream tools treat as
"unknown location".

### `meshcore.send_text_message` action

```yaml
- meshcore.send_text_message:
    id: <hub id>
    channel: "<name>"   # optional; defaults to first configured channel
    text: "<utf-8>"      # required, templatable
```

Wire-compatible with upstream `BaseChatMesh::sendGroupMessage`. Both
fields are templatable, so you can use `!lambda` to splice in sensor
state.

The legacy name `meshcore.send_message` is still accepted as a
deprecated alias.

## Board recipes

### Heltec WiFi LoRa 32 V3 (SX1262)

```yaml
meshcore:
  radio: sx1262
  sclk_pin: GPIO9
  miso_pin: GPIO11
  mosi_pin: GPIO10
  cs_pin: GPIO8
  dio1_pin: GPIO14
  reset_pin: GPIO12
  busy_pin: GPIO13
  tcxo_voltage: 1.6
  dio2_as_rf_switch: true
  rx_boosted_gain: true
```

### TTGO T-Beam (SX1276)

```yaml
i2c:                     # AXP192 PMIC + OLED live on bus 0
  sda: GPIO21
  scl: GPIO22

meshcore:
  radio: sx1276
  sclk_pin: GPIO5
  miso_pin: GPIO19
  mosi_pin: GPIO27
  cs_pin: GPIO18
  dio0_pin: GPIO26
  reset_pin: GPIO23
  # dio1_pin omitted -- T-Beam doesn't wire it; RadioLib polls
  tx_power: 17           # SX1276 PA_BOOST tops out here
```

The T-Beam's AXP192 powers the LoRa radio by default, so no PMIC
component is strictly required. If your radio fails to init with
`status=-2`, the LoRa LDO is the first thing to suspect.

## How it works

- The Python `__init__.py` translates each YAML radio param into the
  compile flags MeshCore's `CustomSX126x` / `CustomSX1276` helpers
  already read (`P_LORA_*`, `LORA_FREQ`, `LORA_BW`, `LORA_SF`,
  `LORA_TX_POWER`, `SX126X_DIO3_TCXO_VOLTAGE`, ...). Avoids forking
  the upstream helpers.
- We bypass `std_init()` and call RadioLib's `begin()` directly so
  init failures surface with a numeric error code in the ESPHome log.
- Build flags enable `RADIOLIB_GODMODE` + `RADIOLIB_STATIC_ONLY` and
  exclude the ham-radio modules MeshCore doesn't use.
- `cg.add_library("SPI"|"Wire"|"Preferences", None)` re-enables those
  Arduino bundled libs, which since [ESPHome 2026.2 are off by
  default](https://developers.esphome.io/blog/2026/02/12/esp32-arduino-selective-compilation-libraries-disabled-by-default/).
- `MeshCoreComponent::setup()` brings up MeshCore's `ESP32Board`,
  `SimpleMeshTables`, `StaticPoolPacketManager`, then constructs a
  `CustomSX1262` / `CustomSX1276` driven by the `SPI` instance.
- Identity (96-byte prv+pub blob) is loaded with priority:
  1. `private_key:` from YAML — pin a deterministic identity across
     reflashes. Recommended pattern: `private_key: !secret ...`.
  2. NVS cache via ESPHome's `global_preferences` — the keypair
     generated on first boot persists across reboots.
  3. Fresh RNG seed and saved to NVS on first boot.
- The local RTC starts at MeshCore's 2024-05-15 baseline and learns
  real time from inbound advert and `PAYLOAD_TYPE_GRP_TXT` timestamps.
  Persisted to NVS so subsequent boots don't fall back to the
  baseline. Off-grid nodes converge their clock without WiFi/NTP/GPS.
- Each YAML `channels[]` entry decodes its PSK, hashes with SHA256 to
  produce the routing path-hash, and is held in a
  `std::vector<ChannelDetails>`. `EsphomeMesh::searchChannelsByHash`
  hands matches back to the dispatcher.
- Inbound `PAYLOAD_TYPE_GRP_TXT` packets land in
  `EsphomeMesh::onGroupDataRecv`, get decoded into the
  `"sender: text"` string, and pushed to the bound `text_sensor`
  along with RSSI/SNR.
- `meshcore.send_text_message` builds the
  `[ts(4)|txt_type(1)|"name: msg\0"]` frame, encrypts it under the
  channel PSK via `Mesh::createGroupDatagram`, and floods it. Wire-
  compatible with any other `BaseChatMesh`-derived node.
- In `role: repeater`, `allowPacketForward` returns true so the
  dispatcher flood-forwards traffic. The node also broadcasts a
  self-advert at boot so other nodes can discover it.
- Trace packets that arrive at this node as their final destination
  are logged with per-hop SNR (useful for debugging mesh topology
  from a phone's "trace route" button). Repeater-mode nodes that see
  trace packets in transit will append their SNR and forward
  automatically — that's the upstream default behaviour we get for
  free.

## Notes on the build

- **Vendored ed25519**: MeshCore's `Identity.cpp` includes
  `<ed_25519.h>` but ships those C files in `lib/ed25519/`, which PIO
  doesn't pull in when MeshCore is consumed as a library. We mirror
  that directory under `libs/ed25519/` with a tiny `library.json`,
  and point `lib_extra_dirs` at it.
- **Verified**: `esphome compile example.yaml` produces a working
  ~960 KB `firmware.bin` for `heltec_wifi_lora_32_V3` (SX1262) and
  ~1.1 MB factory image for `t-beam` (SX1276) including AXP192
  battery template-sensors and GPS UART.

## Roadmap

- [ ] `advert_interval:` for periodic re-advertising in repeater mode
      (boot-only currently).
- [ ] Replace `esp_random()` RNG seed with `RadioNoiseListener` for
      proper entropy at first boot before the radio has been heard
      from.
- [ ] Contact/peer support via `BaseChatMesh` (sender lookup by
      pubkey, ACKs, paths, contact list management).
- [ ] Optional `text_sensor` that publishes the *full* `"sender:
      text"` string vs just `text`, configurable via YAML.
- [ ] Region map / regulatory presets (use upstream `RegionMap.h`).
- [ ] Optional BLE Companion Radio mode mirroring upstream's example.
- [ ] Pin `lib_deps` to a tagged MeshCore release once stable.

## License

This component is MIT (matching MeshCore upstream). The vendored
`libs/ed25519` is public domain (see `libs/ed25519/src/license.txt`,
[orlp/ed25519](https://github.com/orlp/ed25519)).
