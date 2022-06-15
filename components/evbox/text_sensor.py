import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID, CONF_TOTAL_ENERGY
)
from . import CONF_EVBOX_ID, CONF_SAMPLEVALUE, CONF_SETPOINT, EVBOX_COMPONENT_SCHEMA

CONF_CHARGE_CURRENT = "charge_current"
CONF_TOTAL_ENERGY = "total_energy"

AUTO_LOAD = ["evbox"]

TYPES = {
    CONF_TOTAL_ENERGY,
    CONF_SAMPLEVALUE,
    CONF_CHARGE_CURRENT,
}

CONFIG_SCHEMA = EVBOX_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(type): text_sensor.TEXT_SENSOR_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(text_sensor.TextSensor)}
        )
        for type in TYPES
    }
)

async def to_code(config):
    paren = await cg.get_variable(config[CONF_EVBOX_ID])

    for type in TYPES:
        if type in config:
            conf = config[type]
            var = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(var, conf)
            cg.add(getattr(paren, f"set_{type}_text_sensor")(var))
