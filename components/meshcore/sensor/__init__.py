"""sensor platform for the meshcore hub.

Optional sub-sensors:
  rssi             - dBm of the last received packet
  snr              - dB of the last received packet
  battery_voltage  - reported via mesh::MainBoard::getBattMilliVolts()
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_BATTERY_VOLTAGE,
    CONF_ID,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_VOLT,
)

from .. import MeshCoreComponent, meshcore_ns

CONF_MESHCORE_ID = "meshcore_id"
CONF_RSSI = "rssi"
CONF_SNR = "snr"

DEPENDENCIES = ["meshcore"]

MeshCoreSensorBundle = meshcore_ns.class_("MeshCoreSensorBundle", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MeshCoreSensorBundle),
        cv.GenerateID(CONF_MESHCORE_ID): cv.use_id(MeshCoreComponent),
        cv.Optional(CONF_RSSI): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SNR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_MESHCORE_ID])
    cg.add(parent.set_sensor_bundle(var))

    if rssi := config.get(CONF_RSSI):
        cg.add(var.set_rssi_sensor(await sensor.new_sensor(rssi)))
    if snr := config.get(CONF_SNR):
        cg.add(var.set_snr_sensor(await sensor.new_sensor(snr)))
    if batt := config.get(CONF_BATTERY_VOLTAGE):
        cg.add(var.set_battery_sensor(await sensor.new_sensor(batt)))
