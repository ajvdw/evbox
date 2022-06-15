import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.cpp_helpers import gpio_pin_expression
from esphome.components import uart, mqtt
from esphome.const import (
    CONF_ID,
    CONF_FLOW_CONTROL_PIN,
)
from esphome import pins

CONF_SETPOINT = "setpoint"
CONF_MAX_CC = "max_charge_current"
CONF_MIN_CC = "min_charge_current"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_SAMPLETIME = "sampletime"
CONF_SAMPLEVALUE = "samplevalue"

CODEOWNERS = ["@ajvdw"]
DEPENDENCIES = ["uart"]

CONF_EVBOX_ID = "evbox_id"

evbox_ns = cg.esphome_ns.namespace("esphome::mqtt::evbox")
EVBoxDevice = evbox_ns.class_("EVBoxDevice", uart.UARTDevice, cg.Component)

SetSampleValueAction = evbox_ns.class_("SetSampleValueAction", automation.Action)


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
            cv.Optional(CONF_MIN_CC,default=6.0): cv.float_range(min=6.0, max=32.0),
            cv.Optional(CONF_MAX_CC,default=16.0): cv.float_range(min=6.0, max=32.0),
            cv.Optional(CONF_KP,default=0.7): cv.float_,
            cv.Optional(CONF_KI,default=0.1): cv.float_,
            cv.Optional(CONF_KD,default=0.05): cv.float_,
            cv.Optional(CONF_SAMPLETIME,default=1.0): cv.float_,
            cv.Optional(CONF_SETPOINT,default=0.0): cv.float_,
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
    if CONF_MIN_CC in config:
        cg.add(var.set_min_cc(config[CONF_MIN_CC]))
    if CONF_MAX_CC in config:
        cg.add(var.set_max_cc(config[CONF_MAX_CC]))
    if CONF_KP in config:
        cg.add(var.set_kp(config[CONF_KP]))
    if CONF_KI in config:
        cg.add(var.set_ki(config[CONF_KI]))
    if CONF_KD in config:
        cg.add(var.set_kd(config[CONF_KD]))
    if CONF_SAMPLETIME in config:
        cg.add(var.set_sampletime(config[CONF_SAMPLETIME]))
    if CONF_SETPOINT in config:
        cg.add(var.set_setpoint(config[CONF_SETPOINT]))

@automation.register_action(
    "evbox.set_samplevalue",
    SetSampleValue,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(EVBoxDevice),
            cv.Required(CONF_SAMPLEVALUE): cv.float_,
        }
    ),
)
async def evbox_set_samplevalue_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_SAMPLEVALUE], args, cg.float_)
    cg.add(var.set_samplevalue(template_))
    return var