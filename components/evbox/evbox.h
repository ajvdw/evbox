#pragma once

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

static const char* const TAG = "evbox";

namespace esphome {
namespace evbox {

class EVBoxDevice : public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_min_cc(float min_charge_current) { this->min_charge_current_ = min_charge_current; }
  void set_max_cc(float max_charge_current) { this->max_charge_current_ = max_charge_current; }
  void set_sampletime(float sampletime) { this->sampletime_ = sampletime; }
  void set_samplevalue(float samplevalue) { this->samplevalue_ = samplevalue; }


  void set_samplevalue_text_sensor(text_sensor::TextSensor *text_sensor) { this->samplevalue_text_sensor_ = text_sensor };
  void set_charge_current_text_sensor(text_sensor::TextSensor *text_sensor) { this->charge_current_text_sensor_ = text_sensor };
  void set_total_energy_text_sensor(text_sensor::TextSensor *text_sensor) { this->total_energy_text_sensor_ = text_sensor };
      |                 ^~~~~~~~~~~~~~~~~~~~~~~~~~~~


  void set_setpoint(float setpoint) { this->setpoint_ = setpoint; }
  void set_kp(float kp) { this->kp_ = kp; }
  void set_ki(float ki) { this->ki_ = ki; }
  void set_kd(float kd) { this->kd_ = kd; }
 
 protected:
  GPIOPin *flow_control_pin_{nullptr};
  TextSensor *samplevalue_text_sensor_{nullptr};
  TextSensor *set_charge_current_text_sensor_{nullptr};
  TextSensor *total_energy_text_sensor{nullptr};
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

template<typename... Ts> class SetSampleValueAction : public Action<Ts...> {
    public:
    explicit SetSampleValueAction(EVBoxDevice *parent) : parent_(parent) {}

    TEMPLATABLE_VALUE(float, samplevalue);

    void play(Ts... x) override {
        float samplevalue = this->samplevalue_.value(x...);
        this->parent_->set_samplevalue(samplevalue);
    }
    
    protected:
    EVBoxDevice *parent_;
};

}  // namespace evbox
}  // namespace esphome

