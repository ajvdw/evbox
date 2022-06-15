#include "evbox.h"
#include "pid.h"

namespace esphome {
namespace mqtt {
namespace evbox {

PID *pid;

void EVBoxDevice::setup() {
  // Set flowcontrolpin
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();

  ESP_LOGD(TAG, "Setup");

  std::string command_topic = App.get_name() + std::string("/power/input");
  this->subscribe(command_topic, &EVBoxDevice::on_mqtt_receive_);
  ESP_LOGD(TAG, "MQTT subscribed to %s", command_topic.c_str());

  received_len_ = 0;
  receiving_ = false;

  pid = new PID(&(this->samplevalue_), &(this->output_charge_current_), &(this->setpoint_), this->kp_, this->ki_, this->kd_, DIRECT);
  pid->SetSampleTime(sampletime_*1000); // seconds
  pid->SetOutputLimits(min_charge_current_, max_charge_current_);
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
  if( now - lastSample > this->sampletime_ )
  {
    pid->Compute();
    lastSample = now;

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

  // Broadcast verbose state every 5 seconds
  if (now - lastMsg > 5000) {
    lastMsg = now;
    std::string topic = App.get_name() + "/current/state";
    if (this->publish(topic, string(samplevalue_), 0, true))
      ESP_LOGD(TAG, "Success sending MQTT state message");
    else
      ESP_LOGD(TAG, "Error sending MQTT state message");
  }
}

void EVBoxDevice::on_mqtt_receive_(const std::string& topic, const std::string& payload) {
  // do something with topic and payload
  ESP_LOGD(TAG, "Payload %s on topic %s received", payload.c_str(), topic.c_str());
  if (payload == "*REBOOT*")
    ESP.restart();
}


}  // namespace evbox
}  // namespace mqtt
}  // namespace esphome