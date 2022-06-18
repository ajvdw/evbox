import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    ICON_EMPTY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_WATT_HOURS,
)

from . import CONF_EVBOX_ID, EVBOX_COMPONENT_SCHEMA

CONF_CHARGE_CURRENT = "charge_current"
CONF_TOTAL_ENERGY = "total_energy"

AUTO_LOAD = ["evbox"]

TYPES = { CONF_CHARGE_CURRENT: sensor.sensor_schema( UNIT_AMPERE, 
                ICON_EMPTY, 
                1, 
                DEVICE_CLASS_CURRENT, 
                STATE_CLASS_MEASUREMENT
            ),
          CONF_TOTAL_ENERGY: sensor.sensor_schema( UNIT_WATT_HOURS,
                ICON_EMPTY,
                1,
                DEVICE_CLASS_ENERGY,
                STATE_CLASS_TOTAL_INCREASING,
            ),
}

CONFIG_SCHEMA = EVBOX_COMPONENT_SCHEMA.extend(
    {cv.Optional(type): schema for type, schema in TYPES.items()}
)

async def to_code(config):
    paren = await cg.get_variable(config[CONF_EVBOX_ID])

    for type, _ in TYPES.items():
        if type in config:
            conf = config[type]
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(paren, f"set_{type}_sensor")(sens))
