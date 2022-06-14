#pragma once

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/mqtt/custom_mqtt_device.h"

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
  uint8_t receive_data_[256];
  uint32_t received_len_;
  bool receiving_;
};

}  // namespace evbox
}  // namespace mqtt
}  // namespace esphome
