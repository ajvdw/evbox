#pragma once

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
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
  void set_min_cc(float min_charge_current) { this->min_charge_current_ = min_charge_current; }
  void set_max_cc(float max_charge_current) { this->max_charge_current_ = max_charge_current; }
  void set_sampletime(float sampletime) { this->sampletime_ = sampletime; }
  void set_samplevalue(float samplevalue) { this->samplevalue_ = samplevalue; }
  void set_setpoint(float setpoint) { this->setpoint_ = setpoint; }
  void set_kp(float kp) { this->kp_ = kp; }
  void set_ki(float ki) { this->ki_ = ki; }
  void set_kd(float kd) { this->kd_ = kd; }

 template<typename... Ts> class SetSampleValueAction : public Action<Ts...> {
 public:
  explicit SetSampleValueAction(Stepper *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(float, samplevalue);

  void play(Ts... x) override {
    float samplevalue = this->samplevalue.value(x...);
  }

 protected:
  Stepper *parent_;
};
 
 protected:
  void on_mqtt_receive_(const std::string& topic, const std::string& payload);
  GPIOPin *flow_control_pin_{nullptr};

  bool receiving_;
  uint8_t received_data_[256];
  uint32_t received_len_;

  double min_charge_current_;
  double max_charge_current_;
  double output_charge_current_;
  double setpoint_;
  double samplevalue_;
  double sampletime_; 
  double kp_;
  double ki_;
  double kd_;

};

}  // namespace evbox
}  // namespace mqtt
}  // namespace esphome
