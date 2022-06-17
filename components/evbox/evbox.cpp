#include "evbox.h"
#include "pid.h"

namespace esphome {
namespace evbox {

PID *pid;
const char hex[] = "0123456789ABCDEF";

void EVBoxDevice::setup() {
  if (this->flow_control_pin_ != nullptr)
  {
    // Set flowcontrolpin
    this->flow_control_pin_->setup();
    // Flow control to RX
    this->flow_control_pin_->digital_write(0);
  }
  ESP_LOGD(TAG, "Setup");

  received_len_ = 0;
  total_energy_ = 0;
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
      process_message_( (char *)received_data_);
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
    samplevalue_text_sensor_->publish_state( std::to_string(this->samplevalue_).c_str(),1 );

    pid->Compute();
    lastSample = now;

    send_max_current_( this->output_charge_current_);
    charge_current_text_sensor_->publish_state( std::to_string(this->output_charge_current_).c_str(),1 );
  }
}

void EVBoxDevice::process_message_( char *msg )
{
  int i;
  int msglen = strlen(msg);

  // Calc checksum
  if( msglen > 12 )
  {
    int cm = 0; 
    int cx = 0; 
    
    for(i=0; i<msglen-4; i++ )
    {
      cm = (cm + msg[i]) & 255;
      cx = cx ^ (msg[i]);
    }

    uint8_t csm = (uint8_t)cm; 
    uint8_t csx = (uint8_t)cx; 

    // Checksum and header validation
    if( hex[csm >> 4] == msg[msglen-4] && hex[csm & 15] == msg[msglen-3] && 
        hex[csx >> 4] == msg[msglen-2] && hex[csx & 15] == msg[msglen-1] && 
        strncmp( msg, "A08069", 6  ) == 0 )
    {
  
      double m=0;
      long factor = 268435456;
      for( i = msglen-12; i<msglen-4; i++)
      {
        char c = msg[i];
        int v = 0;
        if( c >= '0' && c <= '9')
          v = c -'0';
        if( c >= 'A' && c <= 'F' )
          v = c - 'A' + 10;
        m = m + factor * v;
        factor = factor / 16; 
      }
      this->total_energy_ = m /1000;
      total_energy_text_sensor_->publish_state( std::to_string(this->total_energy_).c_str(),1 );
    }
  }
  //ESP_LOGD(TAG, "RX: %s", msg );
}

void EVBoxDevice::send_max_current_( float amp ) {
  // MaxChargingCurrent command
  char buf[] = "80A06900__00__00__" ;   
  int  ta = round(10*amp);
 
  // Set current values (fill in the blanks)
  buf[8]=buf[12]=buf[16]=hex[(ta >> 4) & 15];
  buf[9]=buf[13]=buf[17]=hex[(ta >> 0) & 15];

  // Calc checksum
  int cs;
  
  cs = 0; 
  for (int i = 0; buf[i]; i++) cs = (cs + buf[i]) & 255;
  uint8_t csm = (uint8_t)cs; 

  cs = 0; 
  for (int i = 0; buf[i]; i++) cs = cs ^ (buf[i]);
  uint8_t csx = (uint8_t)cs; 

  ESP_LOGD(TAG, "Send charge current");

  if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(1); // TX mode 

  // StartOfMessage
  this->write_byte(2);
  // Actual Message
  this->write_array((uint8_t *)buf,18);  
  // Add checksum to message
  this->write_byte(hex[csm >> 4]);
  this->write_byte(hex[csm & 15]);
  this->write_byte(hex[csx >> 4]);
  this->write_byte(hex[csx & 15]);  
  // EndOfMessage
  this->write_byte(3);
  this->flush();

  if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(0); // RX mode 
}

}  // namespace evbox
}  // namespace esphome