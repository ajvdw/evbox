import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.cpp_helpers import gpio_pin_expression
from esphome.components import uart, mqtt
from esphome.const import (
    CONF_ID,
    CONF_FLOW_CONTROL_PIN,
)
from esphome import pins

CODEOWNERS = ["@ajvdw"]
DEPENDENCIES = ["uart"]

CONF_EVBOX_ID = "evbox_id"

evbox_ns = cg.esphome_ns.namespace("esphome::mqtt::evbox")
EVBoxDevice = evbox_ns.class_("EVBoxDevice", uart.UARTDevice, cg.Component)

EVBOX_COMPONENT_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.Required(CONF_EVBOX_ID): cv.use_id(EVBoxDevice),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EVBoxDevice),
            cv.Required(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    cg.add_global(evbox_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))
