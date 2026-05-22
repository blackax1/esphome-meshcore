"""text_sensor platform for the meshcore hub.

Currently exposes a single `last_message` text sensor that gets the payload
of the most recently received CHANNEL packet (UTF-8).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID

from .. import MeshCoreComponent, meshcore_ns

CONF_MESHCORE_ID = "meshcore_id"
CONF_LAST_MESSAGE = "last_message"

DEPENDENCIES = ["meshcore"]

MeshCoreTextSensor = meshcore_ns.class_("MeshCoreTextSensor", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MeshCoreTextSensor),
        cv.GenerateID(CONF_MESHCORE_ID): cv.use_id(MeshCoreComponent),
        cv.Optional(CONF_LAST_MESSAGE): text_sensor.text_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_MESHCORE_ID])
    cg.add(parent.set_last_message_sensor(var))

    if last := config.get(CONF_LAST_MESSAGE):
        sens = await text_sensor.new_text_sensor(last)
        cg.add(var.set_last_message(sens))
