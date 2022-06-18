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

from . import CONF_EVBOX_ID, EVBOX_COMPONENT_SCHEMA, CONF_SAMPLEVALUE

CONF_CALCULATED_CURRENT = "calculated_current"
CONF_REQUESTED_CURRENT = "requested_current"
CONF_PHASE1_CURRENT = "phase1_current"
CONF_PHASE2_CURRENT = "phase2_current"
CONF_PHASE3_CURRENT = "phase3_current"
CONF_TOTAL_ENERGY = "total_energy"

AUTO_LOAD = ["evbox"]

TYPES = { CONF_CALCULATED_CURRENT: sensor.sensor_schema( 
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT, 
            ),
          CONF_REQUESTED_CURRENT: sensor.sensor_schema( 
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT, 
            ),
          CONF_PHASE1_CURRENT: sensor.sensor_schema( 
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT, 
            ),
          CONF_PHASE2_CURRENT: sensor.sensor_schema( 
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT, 
            ),  
          CONF_PHASE3_CURRENT: sensor.sensor_schema( 
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT, 
            ),  
          CONF_TOTAL_ENERGY: sensor.sensor_schema( 
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING, 
            ),
         CONF_SAMPLEVALUE: sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=1,
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
