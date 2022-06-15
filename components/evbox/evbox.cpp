#include "evbox.h"
#include "pid.h"

namespace esphome {
namespace evbox {

PID *pid;

void EVBoxDevice::setup() {
  // Set flowcontrolpin
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();

  ESP_LOGD(TAG, "Setup");

  received_len_ = 0;
  receiving_ = false;

  pid = new PID(&(this->samplevalue_), &(this->output_charge_current_), &(this->setpoint_), this->kp_, this->ki_, this->kd_, DIRECT);
  pid->SetSampleTime(this->sampletime_ * 1000); 
  pid->SetOutputLimits(this->min_charge_current_, this->max_charge_current_);
  pid->SetMode(AUTOMATIC);
}

void EVBoxDevice::loop() {
  const uint32_t now = millis();
  static uint32_t lastSample=now;
  static uint32_t lastMsg=0;


  // Capture EVBox data and just print it
  if (this->available()) {
    uint8_t c;
    this->read_byte(&c);

    if( c == 2 ) { // Message Start  
       receiving_=true;
       received_len_=0;
    }
    else if( c == 3 && received_len_ > 8) { // Message End 
      receiving_=false;
      received_data_[received_len_]=0;
      ESP_LOGD(TAG, "RX: %s", received_data_ );
      received_len_=0;     
    }
    else if( receiving_ && c >= 48 && c<= 70 ) { // Capture message data
      if( received_len_<255) received_data_[received_len_++]=c;
    }
    else { // Invalid data
      received_len_=0;
    }
  }    

  // Take a sample if time has passed
  if( (now - lastSample) > 1000.0*(this->sampletime_ ) )
  {
    samplevalue_text_sensor_->publish_state( std::to_string(this->samplevalue_).c_str() );

    pid->Compute();
    lastSample = now;

    charge_current_text_sensor_->publish_state( std::to_string(this->output_charge_current_).c_str() );
 
    // Send data 

    // Flow control to TX
    if (this->flow_control_pin_ != nullptr)
      this->flow_control_pin_->digital_write(true);
    // this->write_array((uint8_t *) send_data, 10);
    // Wait until complete
    this->flush(); 
    // Flow control to RX
    if (this->flow_control_pin_ != nullptr) 
      this->flow_control_pin_->digital_write(false);
  }
}

}  // namespace evbox
}  // namespace esphome