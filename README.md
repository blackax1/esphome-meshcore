# esphome-meshcore

An [ESPHome](https://esphome.io) external component that embeds the
[MeshCore](https://github.com/meshcore-dev/MeshCore) LoRa mesh library into an
ESP32-based ESPHome node. Lets you build a Home Assistant-connected mesh
endpoint without writing PlatformIO firmware from scratch.

> Status: **builds end-to-end** against ESPHome 2026.5 + arduino-esp32
> 3.3.8 + MeshCore 1.10. SX1262 only for now. Identity persists across
> reboots via NVS. Channel messaging works (sender:msg framing matches
> upstream `BaseChatMesh::sendGroupMessage`). See [Roadmap](#roadmap)
> for what's still open.

## Layout

```
esphome-meshcore/
├── components/
│   └── meshcore/
│       ├── __init__.py          # hub schema + lib_deps + radio defines
│       ├── meshcore.h           # MeshCoreComponent + EsphomeMesh + sub-sensors
│       ├── meshcore.cpp         # setup() / loop() / send_message() / mesh callbacks
│       ├── automation.h         # SendMessageAction
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

1. Drop the repo into your ESPHome YAML:

   ```yaml
   external_components:
     - source: github://YOUR_USER/esphome-meshcore
       components: [meshcore]
   ```

2. Configure the hub against your LoRa module pins (Heltec V3 shown). Note
   we own the SPI bus directly: do not also add an `spi:` block, MeshCore
   sets up `SPI.begin()` internally.

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
     frequency: 869.525  # MHz, region-appropriate
     bandwidth: 250      # kHz
     spreading_factor: 11
     coding_rate: 5
     tx_power: 22        # dBm
     tcxo_voltage: 1.6
     dio2_as_rf_switch: true
     rx_boosted_gain: true
     channels:
       # 16-byte (128-bit) base64 PSK shared by all participants.
       # Generate with: openssl rand 16 | base64
       - name: "public"
         key: "AAECAwQFBgcICQoLDA0ODw=="

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

3. Send a message from a HA automation:

   ```yaml
   on_...:
     then:
       - meshcore.send_message:
           id: mesh_hub
           text: "hello mesh"
   ```

See [`example.yaml`](./example.yaml) for a full file.

## How it works

- The Python `__init__.py` translates each YAML radio param into the
  compile flags MeshCore's `CustomSX1262::std_init()` already reads
  (`P_LORA_*`, `LORA_FREQ`, `LORA_BW`, `LORA_SF`, `LORA_TX_POWER`,
  `SX126X_DIO3_TCXO_VOLTAGE`, ...). This avoids forking the upstream
  helpers.
- Build flags also enable `RADIOLIB_GODMODE` + `RADIOLIB_STATIC_ONLY` and
  exclude the ham-radio modules MeshCore doesn't use.
- `cg.add_library("SPI"|"Wire"|"Preferences", None)` re-enables those
  Arduino bundled libs, which since [ESPHome 2026.2 are off by
  default](https://developers.esphome.io/blog/2026/02/12/esp32-arduino-selective-compilation-libraries-disabled-by-default/).
- `MeshCoreComponent::setup()` brings up MeshCore's `ESP32Board`,
  `SimpleMeshTables`, `StaticPoolPacketManager`, then constructs a
  `CustomSX1262` driven by the `SPI` instance.
- Identity (96-byte prv+pub blob) is loaded with the following priority:
  1. `private_key:` from YAML (hex string, 64 or 96 bytes). Highest
     precedence — gives you a deterministic node pubkey across reflashes
     and across boards. Recommended pattern: `private_key: !secret
     meshcore_private_key`.
  2. NVS cache via ESPHome's `global_preferences`. The keypair generated
     on first boot lives here so reboots don't look like new nodes.
  3. Generated fresh from RNG and saved to NVS on first boot.
- Each YAML `channels[]` entry decodes its base64 PSK at runtime,
  hashes it with SHA256 to produce the routing path-hash, and is held
  in a `std::vector<ChannelDetails>`. `EsphomeMesh::searchChannelsByHash`
  hands matches back to the dispatcher.
- Inbound `PAYLOAD_TYPE_GRP_TXT` packets land in
  `EsphomeMesh::onGroupDataRecv`, get decoded into the `"sender: text"`
  string, and pushed to the bound `text_sensor` along with RSSI/SNR.
- `meshcore.send_message` builds the `[ts(4)|txt_type(1)|"name: msg\0"]`
  frame, encrypts it under the channel PSK via
  `Mesh::createGroupDatagram`, and floods it. Wire-compatible with any
  other `BaseChatMesh`-derived node.

## Notes on the build

- **Vendored ed25519**: MeshCore's `Identity.cpp` includes
  `<ed_25519.h>` but ships those C files in `lib/ed25519/`, which PIO
  doesn't pull in when MeshCore is consumed as a library. We mirror
  that directory under `libs/ed25519/` with a tiny `library.json`, and
  point `lib_extra_dirs` at it.
- **Verified**: `esphome compile example.yaml` produces a working ~960
  KB `firmware.bin` for `heltec_wifi_lora_32_V3`.

## Roadmap

- [ ] Replace `esp_random()` RNG seed with `RadioNoiseListener` for
      proper entropy at first boot before the radio has been heard from.
- [ ] Contact/peer support via `BaseChatMesh` (sender lookup by pubkey,
      ACKs, paths, contact list management).
- [ ] Optional `text_sensor` that publishes the *full* `"sender: text"`
      string vs just `text`, configurable via YAML.
- [ ] Region map / regulatory presets (use upstream `RegionMap.h`).
- [ ] More radios: SX1276, LR1110, STM32WLx (today only SX1262).
- [ ] Optional BLE Companion Radio mode mirroring upstream's example.
- [ ] Pin `lib_deps` to a tagged MeshCore release once stable.

## License

This component is MIT (matching MeshCore upstream). The vendored
`libs/ed25519` is public domain (see `libs/ed25519/src/license.txt`,
[orlp/ed25519](https://github.com/orlp/ed25519)).
