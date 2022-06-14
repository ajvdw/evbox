#include "evbox.h"
#include "pid.h"

namespace esphome {
namespace mqtt {
namespace evbox {

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
}

void EVBoxDevice::loop() {
  uint32_t inactivity_seconds = 20;
  static uint32_t lastMsg = 0;

  // Flow control to TX
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  // Send data
  // this->write_array((uint8_t *) send_data, 10);
  // Wait until complete
  this->flush(); 

  // Flow control to RX
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  // Wait for 100ms for data to arrive
  const uint32_t now = millis();
  bool datawaiting = false;
  while (millis() - now < 100 && !datawaiting)
    datawaiting = this->available();

  ESP_LOGV(TAG, "Waited for %d ms for data to arrive", millis() - now);

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
      receive_data_[received_len_]=0;
      ESP_LOGD(TAG, "RX: %s ", receive_data_.c_str());
      received_len_=0;     
    }
    else if( receiving_ && c >= 48 && c<= 70 ) { // Capture message data
      if( received_len_<255) receive_data_[received_len_++]=c;
    }
    else { // Invalid data
      received_len_=0;
    }
  }    

  // Broadcast verbose state every 5 seconds
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();
    std::string topic = App.get_name() + "/current/state";
    if (this->publish(topic, "24", 0, true))
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