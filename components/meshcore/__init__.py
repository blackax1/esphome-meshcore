"""ESPHome external component: MeshCore LoRa mesh hub.

Owns the singleton MeshCore stack on the device and exposes it to other
platforms (sensor, text_sensor) via a shared ID.

The MeshCore library expects a pile of compile-time defines (P_LORA_*,
LORA_FREQ, LORA_BW, LORA_SF, LORA_TX_POWER, ...) that its CustomSX1262
helper reads. We translate the YAML config into those defines so we can
reuse MeshCore's helpers verbatim instead of re-implementing radio init.
"""

from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.core import CORE
from esphome.const import (
    CONF_CS_PIN,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_MISO_PIN,
    CONF_MOSI_PIN,
    CONF_RESET_PIN,
    CONF_TEXT,
    CONF_LATITUDE,
    CONF_LONGITUDE,
)

CODEOWNERS = ["@yourgithub"]
# Note: we deliberately do NOT depend on the `spi` component. MeshCore's
# CustomSX1262 helper drives the SPI bus itself via SPIClass.begin(sclk,
# miso, mosi). Letting ESPHome's spi component own the bus and then handing
# raw pin numbers to RadioLib leads to double-init and wedged transfers.
DEPENDENCIES = []
AUTO_LOAD = ["sensor", "text_sensor"]
MULTI_CONF = False

meshcore_ns = cg.esphome_ns.namespace("meshcore")
MeshCoreComponent = meshcore_ns.class_("MeshCoreComponent", cg.Component)
SendTextMessageAction = meshcore_ns.class_("SendTextMessageAction", automation.Action)
SendSelfAdvertAction = meshcore_ns.class_("SendSelfAdvertAction", automation.Action)


CONF_RADIO = "radio"
CONF_ROLE = "role"
CONF_ADVERT_INTERVAL = "advert_interval"
CONF_SCLK_PIN = "sclk_pin"
CONF_DIO0_PIN = "dio0_pin"
CONF_DIO1_PIN = "dio1_pin"
CONF_BUSY_PIN = "busy_pin"
CONF_BANDWIDTH = "bandwidth"
CONF_TCXO_VOLTAGE = "tcxo_voltage"
CONF_DIO2_AS_RF_SWITCH = "dio2_as_rf_switch"
CONF_RX_BOOSTED_GAIN = "rx_boosted_gain"
CONF_SPREADING_FACTOR = "spreading_factor"
CONF_CODING_RATE = "coding_rate"
CONF_TX_POWER = "tx_power"
CONF_NODE_NAME = "node_name"
CONF_BATTERY_PIN = "battery_pin"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_OWNER_INFO = "owner_info"
CONF_CHANNELS = "channels"
CONF_CHANNEL = "channel"
CONF_KEY = "key"
CONF_PRIVATE_KEY = "private_key"
CONF_GPS_LATITUDE = "gps_latitude"
CONF_GPS_LONGITUDE = "gps_longitude"
CONF_OWNER_NAME = "owner_name"
CONF_OWNER_SERIAL = "owner_serial"
CONF_OWNER_MODEL = "owner_model"
CONF_FIRMWARE_VERSION_STRING = "firmware_version"


def _validate_channel_key(value):
    """Validate a channel PSK and normalise to base64.

    Accepts:
    * base64 string that decodes to 16 or 32 bytes (with or without
      trailing ``=`` padding).
    * hex string of 32 or 64 chars (16 or 32 bytes).

    Returns the canonical base64 representation that the C++ side will
    decode at runtime. Raises cv.Invalid with a message that points the
    user at the right format when the input doesn't match either form.
    """
    import base64
    import binascii

    s = cv.string_strict(value).strip()
    stripped = s.lower()

    # Hex first: it's an unambiguous superset (no '+', '/', '=' etc).
    if all(c in "0123456789abcdef" for c in stripped) and len(stripped) in (32, 64):
        raw = binascii.unhexlify(stripped)
        return base64.b64encode(raw).decode("ascii")

    # Otherwise try base64. Pad if needed so outputs from
    # "openssl rand 16 | base64" with no trailing newlines still parse.
    padded = s + "=" * (-len(s) % 4)
    try:
        raw = base64.b64decode(padded, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise cv.Invalid(
            f"channel key {s!r} is not valid base64 or hex: {exc}"
        ) from exc
    if len(raw) not in (16, 32):
        raise cv.Invalid(
            f"channel key must decode to 16 or 32 bytes (128- or 256-bit "
            f"PSK); got {len(raw)} bytes from {s!r}"
        )
    return base64.b64encode(raw).decode("ascii")


def _validate_identity_hex(value):
    """Hex string for a MeshCore Ed25519 private key.

    Accepts either 64 bytes (just prv_key, pub_key derived on device) or
    96 bytes (prv_key + pub_key concatenated, matching the on-device NVS
    layout).
    """
    s = cv.string_strict(value).strip().lower()
    if any(c not in "0123456789abcdef" for c in s):
        raise cv.Invalid("identity.private_key must be a hex string")
    if len(s) not in (128, 192):
        raise cv.Invalid(
            "identity.private_key must decode to 64 bytes "
            "(128 hex chars, prv_key only) or 96 bytes "
            f"(192 hex chars, prv_key + pub_key); got {len(s)} chars"
        )
    return s

# Map YAML radio name -> the macro MeshCore's helpers gate on.
RADIO_TYPES = {
    "sx1262": "USE_SX1262",
    "sx1276": "USE_SX1276",
    "sx1278": "USE_SX1276",  # 1278 is a re-banded 1276; same driver
}

# Radios in the SX126x family use a BUSY pin and support TCXO / DIO2-as-
# RF-switch / boosted RX gain. The SX127x family uses DIO0 instead of
# BUSY and has none of those knobs. Splitting here keeps the YAML clean
# and keeps the C++ side from wiring up irrelevant options.
SX126X_RADIOS = {"sx1262"}
SX127X_RADIOS = {"sx1276", "sx1278"}

# Node behaviour roles.
#
# - companion: hear traffic, send when asked, do NOT retransmit. This is
#   what most ESPHome users want — a leaf node attached to Home Assistant
#   that emits sensor readings and surfaces incoming messages but
#   doesn't add load to the mesh.
# - repeater: companion behaviour + flood-forward every received packet
#   that we haven't already seen, up to MeshCore's normal hop limit. Use
#   when the device is a static, well-powered relay covering a gap
#   between other nodes. Adds duty-cycle pressure on the channel.
ROLES = {
    "companion": "ROLE_COMPANION",
    "repeater": "ROLE_REPEATER",
}


CHANNEL_SCHEMA = cv.Schema(
    {
        cv.Required("name"): cv.string_strict,
        # Channel PSK. Accept either base64 (matching upstream MeshCore
        # tools, e.g. izOH6cXN6mrJ5e26oRXNcg==) or raw hex (matching
        # `openssl rand -hex 16`). Either form must decode to 16 or 32
        # bytes. Hex inputs are normalised to base64 here so the C++
        # side can stay base64-only.
        cv.Required(CONF_KEY): _validate_channel_key,
    }
)


def _validate_framework(value):
    """Reject non-Arduino frameworks at config time with a clear message.

    Has to live as a top-level validator (run after CORE.using_arduino is
    populated) rather than in to_code, because to_code only runs during
    `esphome compile`, not `esphome config`.
    """
    if not (CORE.is_esp32 and CORE.using_arduino):
        raise cv.Invalid(
            "meshcore requires the Arduino framework on ESP32. Add this "
            "to your YAML:\n\n"
            "  esp32:\n"
            "    framework:\n"
            "      type: arduino\n\n"
            "MeshCore and RadioLib pull in <SPI.h>, <Arduino.h>, "
            "digitalRead/Write, and attachInterrupt, which are not "
            "available under the esp-idf framework."
        )
    return value


def _validate_radio_pins(config):
    """Per-radio sanity checks on which pins must / must not be set."""
    radio = config[CONF_RADIO]
    has_busy = CONF_BUSY_PIN in config
    has_dio0 = CONF_DIO0_PIN in config
    has_dio1 = CONF_DIO1_PIN in config
    if radio in SX126X_RADIOS:
        if not has_busy:
            raise cv.Invalid(
                f"radio: {radio} requires busy_pin (the SX126x BUSY line). "
                "If your board has no BUSY pin, you probably want radio: "
                "sx1276 or sx1278 instead."
            )
        if not has_dio1:
            raise cv.Invalid(
                f"radio: {radio} requires dio1_pin (the only IRQ line on "
                "the SX126x family)."
            )
        if has_dio0:
            raise cv.Invalid(
                f"radio: {radio} does not use dio0_pin. The SX126x family "
                "uses DIO1 + BUSY only; remove dio0_pin from your config."
            )
    elif radio in SX127X_RADIOS:
        if not has_dio0:
            raise cv.Invalid(
                f"radio: {radio} requires dio0_pin (the SX127x packet-done "
                "interrupt line)."
            )
        if has_busy:
            raise cv.Invalid(
                f"radio: {radio} does not use busy_pin. The SX127x family "
                "uses DIO0 + DIO1 only; remove busy_pin from your config."
            )
        # dio1_pin is intentionally optional for SX127x. Many boards
        # (T-Beam classic, Heltec V1) don't wire DIO1 at all. RadioLib
        # accepts RADIOLIB_NC and falls back to polling.
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MeshCoreComponent),
            cv.Required(CONF_RADIO): cv.enum(RADIO_TYPES, lower=True),
            cv.Required(CONF_SCLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Required(CONF_MISO_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_MOSI_PIN): pins.internal_gpio_output_pin_number,
            cv.Required(CONF_CS_PIN): pins.internal_gpio_output_pin_number,
            cv.Required(CONF_RESET_PIN): pins.internal_gpio_output_pin_number,
            # busy_pin is for SX126x, dio0_pin is for SX127x. We accept
            # either at parse time and require the right one for the
            # chosen radio in _validate_radio_pins below. dio1_pin is
            # required on SX126x (only IRQ line) but optional on SX127x
            # (many boards don't wire it).
            cv.Optional(CONF_DIO1_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_BUSY_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_DIO0_PIN): pins.internal_gpio_input_pin_number,
            # Defaults match upstream MeshCore's platformio.ini, so a
            # node configured with just radio + pins talks on the
            # public mesh out of the box. Override frequency for
            # non-EU868 regions (US/AU/etc).
            cv.Optional(CONF_FREQUENCY, default=869.618): cv.float_range(
                min=137.0, max=1020.0
            ),
            cv.Optional(CONF_BANDWIDTH, default=62.5): cv.one_of(
                7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500, float=True
            ),
            cv.Optional(CONF_SPREADING_FACTOR, default=8): cv.int_range(min=5, max=12),
            cv.Optional(CONF_CODING_RATE, default=5): cv.int_range(min=5, max=8),
            cv.Optional(CONF_TX_POWER, default=17): cv.int_range(min=-9, max=22),
            # SX126x-only knobs. Ignored on SX127x.
            cv.Optional(CONF_TCXO_VOLTAGE, default=1.6): cv.float_range(min=0.0, max=3.3),
            cv.Optional(CONF_DIO2_AS_RF_SWITCH, default=False): cv.boolean,
            cv.Optional(CONF_RX_BOOSTED_GAIN, default=False): cv.boolean,
            cv.Optional(CONF_NODE_NAME, default="esphome-mesh"): cv.string_strict,
            cv.Optional(CONF_ROLE, default="companion"): cv.enum(ROLES, lower=True),
            # How often a repeater re-broadcasts its self-advert. 0 =
            # advert only at boot. Companion role ignores this.
            cv.Optional(
                CONF_ADVERT_INTERVAL, default="0s"
            ): cv.positive_time_period_seconds,
            cv.Optional(CONF_BATTERY_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_PRIVATE_KEY): _validate_identity_hex,
            cv.Optional(CONF_CHANNELS, default=[]): cv.ensure_list(CHANNEL_SCHEMA),
            cv.Optional(CONF_GPS_LATITUDE): cv.float_range(min=-90, max=90),
            cv.Optional(CONF_GPS_LONGITUDE): cv.float_range(min=-180, max=180),
            cv.Optional(CONF_FIRMWARE_VERSION_STRING): cv.string_strict,
            cv.Optional(CONF_OWNER_NAME): cv.string_strict,
            cv.Optional(CONF_OWNER_SERIAL): cv.string_strict,
            cv.Optional(CONF_OWNER_MODEL): cv.string_strict,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_framework,
    _validate_radio_pins,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_node_name(config[CONF_NODE_NAME]))
    cg.add(var.set_repeater(config[CONF_ROLE] == "repeater"))
    cg.add(var.set_advert_interval(config[CONF_ADVERT_INTERVAL].total_seconds))

    if (private_key := config.get(CONF_PRIVATE_KEY)) is not None:
        cg.add(var.set_static_identity(private_key))

    for channel in config[CONF_CHANNELS]:
        cg.add(var.add_channel(channel["name"], channel[CONF_KEY]))

    if CONF_GPS_LATITUDE in config and CONF_GPS_LONGITUDE in config:
        cg.add(var.set_gps_location(config[CONF_GPS_LATITUDE], config[CONF_GPS_LONGITUDE]))

    # Translate YAML config into the macros MeshCore's CustomSX126x /
    # CustomSX1276 helpers expect. Doing this at the build-flag level
    # (vs. poking values in via setters) means the upstream helpers
    # compile without modification.
    radio = config[CONF_RADIO]
    flags = {
        # Pin map shared across both radio families.
        "P_LORA_SCLK": config[CONF_SCLK_PIN],
        "P_LORA_MISO": config[CONF_MISO_PIN],
        "P_LORA_MOSI": config[CONF_MOSI_PIN],
        "P_LORA_NSS": config[CONF_CS_PIN],
        # DIO1 is required on SX126x and optional on SX127x. When the
        # user doesn't supply it (T-Beam-class boards that don't wire
        # DIO1), pass RadioLib's "not connected" sentinel so the driver
        # falls back to polling instead of waiting for an interrupt
        # that's never going to fire.
        "P_LORA_DIO_1": config.get(CONF_DIO1_PIN, "RADIOLIB_NC"),
        "P_LORA_RESET": config[CONF_RESET_PIN],
        # Radio params consumed by std_init.
        "LORA_FREQ": f"{config[CONF_FREQUENCY]:.4f}f",
        "LORA_BW": f"{config[CONF_BANDWIDTH]:.2f}f",
        "LORA_SF": config[CONF_SPREADING_FACTOR],
        "LORA_CR": config[CONF_CODING_RATE],
        "LORA_TX_POWER": config[CONF_TX_POWER],
        # Suppress MeshCore's ESP32Board::begin() Wire.begin(). MeshCore
        # only needs I2C if you're hooking up an RV-3028 RTC, which we
        # don't (our RTC is the ESP32 internal clock). On boards where
        # ESPHome's i2c: component has already claimed bus 0 (T-Beam
        # AXP192/AXP2101 + OLED) the redundant Wire.begin() throws
        # ESP_ERR_INVALID_STATE and aborts board init. -1 takes the
        # no-op branch in ESP32Board.h.
        "PIN_BOARD_SDA": -1,
        "PIN_BOARD_SCL": -1,
    }

    if radio in SX126X_RADIOS:
        # SX126x uses BUSY + DIO1, supports TCXO + DIO2-as-RF-switch +
        # boosted RX gain.
        flags["P_LORA_BUSY"] = config[CONF_BUSY_PIN]
        flags["SX126X_DIO3_TCXO_VOLTAGE"] = f"{config[CONF_TCXO_VOLTAGE]:.2f}f"
        if config[CONF_DIO2_AS_RF_SWITCH]:
            flags["SX126X_DIO2_AS_RF_SWITCH"] = "true"
        if config[CONF_RX_BOOSTED_GAIN]:
            flags["SX126X_RX_BOOSTED_GAIN"] = "true"
    elif radio in SX127X_RADIOS:
        # SX127x uses DIO0 (packet-done IRQ) + DIO1. No TCXO knob.
        flags["P_LORA_DIO_0"] = config[CONF_DIO0_PIN]

    if (battery_pin := config.get(CONF_BATTERY_PIN)) is not None:
        flags["PIN_VBAT_READ"] = battery_pin

    for name, value in flags.items():
        cg.add_build_flag(f"-D{name}={value}")

    # Variant gate (USE_SX1262 / USE_SX1276) consumed by MeshCore's
    # CustomSX126xWrapper / CustomSX1276Wrapper headers.
    cg.add_build_flag(f"-D{RADIO_TYPES[radio]}=1")

    # MeshCore reaches into RadioLib's private members (e.g. SX126x::mod,
    # spreadingFactor, freqMHz). Upstream's platformio.ini sets these
    # build flags; without them compilation of CustomSX1262Wrapper.h and
    # SX126xReset.h fails with "private within this context" errors.
    radiolib_flags = [
        "RADIOLIB_GODMODE=1",
        "RADIOLIB_STATIC_ONLY=1",
        "RADIOLIB_EXCLUDE_CC1101=1",
        "RADIOLIB_EXCLUDE_RF69=1",
        "RADIOLIB_EXCLUDE_SX1231=1",
        "RADIOLIB_EXCLUDE_SI443X=1",
        "RADIOLIB_EXCLUDE_RFM2X=1",
        "RADIOLIB_EXCLUDE_SX128X=1",
        "RADIOLIB_EXCLUDE_AFSK=1",
        "RADIOLIB_EXCLUDE_AX25=1",
        "RADIOLIB_EXCLUDE_HELLSCHREIBER=1",
        "RADIOLIB_EXCLUDE_MORSE=1",
        "RADIOLIB_EXCLUDE_APRS=1",
        "RADIOLIB_EXCLUDE_BELL=1",
        "RADIOLIB_EXCLUDE_RTTY=1",
        "RADIOLIB_EXCLUDE_SSTV=1",
    ]
    for flag in radiolib_flags:
        cg.add_build_flag(f"-D{flag}")

    # Pull MeshCore + transitive deps from PlatformIO. Listing them
    # explicitly because PIO doesn't always honour MeshCore's
    # library.json transitive deps when ESPHome's mixed
    # arduino+espidf build is in play. Also pull arduino-esp32's FS
    # bundled libraries so SimpleMeshTables.h's `#include <FS.h>`
    # resolves; without this the espidf side of the build can't see
    # the Arduino FS headers.
    cg.add_platformio_option(
        "lib_deps",
        [
            "https://github.com/meshcore-dev/MeshCore.git",
            "jgromes/RadioLib@^7.6.0",
            "rweather/Crypto@^0.4.0",
            "adafruit/RTClib@^2.1.3",
            "melopero/Melopero RV3028@^1.1.0",
            "electroniccats/CayenneLPP@1.6.1",
            "densaugeo/base64@^1.4.0",
        ],
    )
    cg.add_platformio_option("lib_ldf_mode", "deep+")
    # MeshCore's Identity.cpp #include's <ed_25519.h>, but the upstream
    # library doesn't ship those C files in its src/. We vendor the
    # reference orlp/ed25519 implementation under libs/ed25519 next to
    # this component, structured as a proper PlatformIO library, and
    # point lib_extra_dirs there so PIO discovers it automatically.
    ed25519_lib_path = (
        Path(__file__).resolve().parent.parent.parent / "libs"
    )
    cg.add_platformio_option("lib_extra_dirs", [str(ed25519_lib_path)])

    # Re-enable the Arduino bundled libraries we need. Since ESPHome
    # 2026.2 the ESP32 Arduino build disables all of them by default and
    # external components must opt back in. None for the version means
    # "use whatever the framework bundles".
    # See https://developers.esphome.io/blog/2026/02/12/esp32-arduino-selective-compilation-libraries-disabled-by-default/
    # SPI for the LoRa radio bus, Wire for I2C peripherals MeshCore
    # helpers may probe (RTC), Preferences for our identity NVS blob.
    # WiFi is only required when the user's YAML uses wifi: with our
    # component active in the same build — adding it unconditionally
    # is harmless because cg.add_library is idempotent and the wifi
    # component already calls add_library("WiFi") itself anyway.
    for arduino_lib in ("SPI", "Wire", "Preferences"):
        cg.add_library(arduino_lib, None)

    # Defensive: arduino-esp32 3.x split Network out of WiFi. ESPHome's
    # wifi component should auto-resolve Network when it calls
    # add_library("WiFi"), but we've seen it miss in some 2026.x
    # releases. Calling add_library("Network") explicitly here forces
    # CONFIG_ARDUINO_SELECTIVE_Network=y in the generated sdkconfig,
    # which makes the bundled library actually compile. It's a no-op
    # if the wifi component already enabled it.
    cg.add_library("Network", None)


# meshcore.send_text_message action.
#
# `text` is required and templatable. `channel` is optional; when omitted
# the message goes out on the first configured channel. We keep the old
# `meshcore.send_message` name registered as an alias for back-compat so
# YAML written against earlier revisions of this component still parses.
MESHCORE_SEND_TEXT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(MeshCoreComponent),
        cv.Required(CONF_TEXT): cv.templatable(cv.string_strict),
        cv.Optional(CONF_CHANNEL, default=""): cv.templatable(cv.string_strict),
    }
)


async def _send_text_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    text = await cg.templatable(config[CONF_TEXT], args, cg.std_string)
    cg.add(var.set_text(text))
    channel = await cg.templatable(config[CONF_CHANNEL], args, cg.std_string)
    cg.add(var.set_channel(channel))
    return var


# New canonical name.
automation.register_action(
    "meshcore.send_text_message",
    SendTextMessageAction,
    MESHCORE_SEND_TEXT_SCHEMA,
    synchronous=True,
)(_send_text_action_to_code)

# --- meshcore.send_self_advert action ---

SEND_SELF_ADVERT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(MeshCoreComponent),
    }
)


async def _send_self_advert_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    return var


automation.register_action(
    "meshcore.send_self_advert",
    SendSelfAdvertAction,
    SEND_SELF_ADVERT_SCHEMA,
    synchronous=True,
)(_send_self_advert_action_to_code)

# Legacy alias kept for back-compat. Deprecated; will be removed in a
# future release.
automation.register_action(
    "meshcore.send_message",
    SendTextMessageAction,
    MESHCORE_SEND_TEXT_SCHEMA,
    synchronous=True,
)(_send_text_action_to_code)

