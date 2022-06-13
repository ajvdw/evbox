#include "esphome/core/log.h"
#include "evbox.h"

namespace esphome {
namespace mqtt {
namespace evbox {

void EVBoxDevice::setup() {
  this->init();
  // Set flowcontrolpin
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->setup();

  ESP_LOGD(TAG, "Setup");

  std::string command_topic = App.get_name() + std::string("/alarm/input");
  this->subscribe(command_topic, &EVBoxDevice::on_mqtt_receive_);
  ESP_LOGD(TAG, "MQTT subscribed to %s", command_topic.c_str());
}

bool void EVBoxDevice::send_(uint8_t address, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3,
                               uint8_t param4, uint8_t param5, uint8_t param6) {
  uint8_t send_data[10];
  send_data[0] = address;
  send_data[1] = param0;
  send_data[2] = param1;
  send_data[3] = param2;
  send_data[4] = param3;
  send_data[5] = param4;
  send_data[6] = param5;
  send_data[7] = param6;

  // Calculate CRC16
  uint8_t bcc_lo = 0xFF;
  uint8_t bcc_hi = 0xFF;
  for (int i = 0; i < 8; i++) {
    uint8_t new_b = send_data[i] ^ bcc_lo;
    uint8_t tmp_b = new_b << 4;
    new_b = tmp_b ^ new_b;
    tmp_b = new_b >> 5;
    bcc_lo = bcc_hi;
    bcc_hi = new_b ^ tmp_b;
    tmp_b = new_b << 3;
    bcc_lo = bcc_lo ^ tmp_b;
    tmp_b = new_b >> 4;
    bcc_lo = bcc_lo ^ tmp_b;
  }
  // CRC bytes
  send_data[8] = (uint8_t)(~bcc_lo);
  send_data[9] = (uint8_t)(~bcc_hi);

  // Clear data
  memset((uint8_t *) receive_data_, 0, 8);
  // Empty RX buffer
  while (this->available()) {
    uint8_t purge;
    this->read_byte(&purge);
  }

  // Flow control to TX
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  // Send data
  this->write_array((uint8_t *) send_data, 10);
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
    if (this->read_array((uint8_t *) receive_data_, 8)) {
      // Calc CRC16
      bcc_lo = 0xFF;
      bcc_hi = 0xFF;
      for (int i = 0; i < 6; i++) {
        uint8_t new_b = receive_data_[i] ^ bcc_lo;
        uint8_t tmp_b = new_b << 4;
        new_b = tmp_b ^ new_b;
        tmp_b = new_b >> 5;
        bcc_lo = bcc_hi;
        bcc_hi = new_b ^ tmp_b;
        tmp_b = new_b << 3;
        bcc_lo = bcc_lo ^ tmp_b;
        tmp_b = new_b >> 4;
        bcc_lo = bcc_lo ^ tmp_b;
      }
      // Check CRC16
      if (receive_data_[7] == (uint8_t)(~bcc_hi) && receive_data_[6] == (uint8_t)(~bcc_lo))
        return true;  // Checksum OK
      else
        ESP_LOGD(TAG, "CRC error in received data");
    } else
      ESP_LOGD(TAG, "Failed receiving data");
  } else
    ESP_LOGD(TAG, "No data available");

  return false;
}


void EVBoxDevice::loop() {
  uint32_t inactivity_seconds = 20;

  // Broadcast verbose state every 5 seconds
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();
    std::string verbose_topic = App.get_name() + "/alarm/verbose_state";
    if (this->publish(verbose_topic, this->GetVerboseState(), 0, true))
      ESP_LOGD(TAG, "Success sending MQTT verbose state message");
    else
      ESP_LOGD(TAG, "Error sending MQTT verbose state message");
  }
}

void EVBoxDevice::on_mqtt_receive_(const std::string& topic, const std::string& payload) {
  // do something with topic and payload
  ESP_LOGD(TAG, "Payload %s on topic %s received", payload.c_str(), topic.c_str());
  if (payload == "DISARM")
    this->sendCommand(Pmax_DISARM);
  else if (payload == "ARM_HOME")
    this->sendCommand(Pmax_ARMHOME);
  else if (payload == "ARM_AWAY")
    this->sendCommand(Pmax_ARMAWAY);
  else if (payload == "*REBOOT*")
    ESP.restart();
}

void EVBoxDevice::mqtt_send_(const char* ZoneOrEvent, const char* WhoOrState, const unsigned char zoneID,
                               int zone_or_system_update) {
  char zoneIDtext[10];
  itoa(zoneID, zoneIDtext, 10);

  // Here we have a zone status change so put this information into JSON
  std::string message_text;
  message_text += "{\"zone_id\": \"";
  message_text += zoneIDtext;
  message_text += "\",\"zone_name\": \"";
  message_text += ZoneOrEvent;
  message_text += "\",\"zone_status\": \"";
  message_text += WhoOrState;
  message_text += "\"";
  message_text += "}";

  // Send zone state
  std::string zone_state_topic = App.get_name() + "/zone/state/" + zoneIDtext;
  if (this->publish(zone_state_topic, message_text, 0, true))
    ESP_LOGD(TAG, "Success sending MQTT zone state message");
  else
    ESP_LOGD(TAG, "Error sending MQTT zone state message");
}

}  // namespace evbox
}  // namespace mqtt
}  // namespace esphome