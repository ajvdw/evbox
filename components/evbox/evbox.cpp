#include "evbox.h"

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
}

void EVBoxDevice::loop() {
  uint32_t inactivity_seconds = 20;
  uint32_t lastMsg = 0;

  // Purge data
  while (this->available()) {
    uint8_t purge;
    this->read_byte(&purge);
  }

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

  // Data waiting?
  if (datawaiting) {
    // Receive data
 /*
    if (this->read_array((uint8_t *) receive_data_, 8)) {
      // Calc CRC16
      // Check CRC16
        return true;  // Checksum OK
    } else
      ESP_LOGD(TAG, "Failed receiving data");
  */
  } else
    ESP_LOGD(TAG, "No data available");


  // Broadcast verbose state every 5 seconds
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();
    std::string topic = App.get_name() + "/current/state";
    if (this->publish(topic, "24", 0, true))
      ESP_LOGD(TAG, "Success sending MQTT verbose state message");
    else
      ESP_LOGD(TAG, "Error sending MQTT verbose state message");
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