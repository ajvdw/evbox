#pragma once

#include "esphome/components/mqtt/custom_mqtt_device.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char* const TAG = "evbox";

namespace esphome {
namespace mqtt {
namespace evbox {

class EVBoxDevice : public uart::UARTDevice, public mqtt::CustomMQTTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  
 protected:
  GPIOPin *flow_control_pin_{nullptr};
  void on_mqtt_receive_(const std::string& topic, const std::string& payload);
};

}  // namespace evbox
}  // namespace mqtt
}  // namespace esphome
