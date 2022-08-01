# EVBox Charge Controller

Homeassistant integration for EVBOX.
Provides functionality for loadbalancing and solarcharging

Use RS485 to communicate with EVBox

```
substitutions:
  device_name: "laadpunt"
  device_description: "EVBox"
     
esphome:
  name: ${device_name}
  comment: "${device_description}"
  platform: ESP8266
  board: d1_mini
  name_add_mac_suffix: false

external_components:
  #use from main branch in GitHub
  - source: "github://ajvdw/evbox"
    components: [evbox]
    refresh: 0s

# Enable logging
logger:
  level: WARN
  
uart:
  rx_pin: GPIO12
  tx_pin: GPIO14
  baud_rate: 38400

select:
  - platform: template
    name: "Operation mode"
    optimistic: true
    options:
      - "Off"
      - "Min"
      - "Wait"
      - "Solar"
      - "Max"
      - "On"
    initial_option: "Solar"
    on_value:
      then:
        evbox.set_mode:
          id: chargepoint1
          mode: !lambda "return (i);"
    
#kp was 0.7    
evbox:
  id: chargepoint1
  flow_control_pin: GPIO13
  sampletime: 1.0
  setpoint: 0.2
  min_charge_current: 6.0
  max_charge_current: 16.0
  kp: 0.25
  ki: 0.1
  kd: 0.05

api:
  services:
    - service: set_charging_to_solar
      then:
        evbox.set_mode:
          id: chargepoint1
          mode: "Solar"
    - service: set_charging_to_wait
      then:
        evbox.set_mode:
          id: chargepoint1
          mode: "Wait"          
          
sensor:
  - platform: homeassistant
    entity_id: sensor.netpower
    id: ha_netpwr
    on_value:
      then:
        evbox.set_samplevalue:
          id: chargepoint1
          samplevalue: !lambda "return 1.0*(x);"
  - platform: evbox
    evbox_id: chargepoint1
    calculated_current:
      name: "EVSE Calculated Current"
    phase1_current:
      name: "EVSE Phase1 Current"
    phase2_current:
      name: "EVSE Phase2 Current"
    phase3_current:
      name: "EVSE Phase3 Current"
    total_energy:
      name: "EVSE Total Energy"
    samplevalue:
      name: "Grid Net power"
      unit_of_measurement: "kW"
          
text_sensor:
  - platform: version
    name: "ESPHome Version ${device_name}"
    hide_timestamp: true

# Enable Webservice
web_server:
  port: 80

#Allow over-the-air updates 
ota:
  id: ota_id
  password: !secret ota_password

#Wifi settings
wifi:
  ssid: !secret wifi_ssid 
  password: !secret wifi_password 
  manual_ip:
    static_ip: !secret ip_laadpunt
    gateway: !secret ip_gateway
    subnet: !secret ip_subnet

#Enable fallback hotspot (captive portal) in case wifi connection fails
  ap:
    ssid: ${device_name}

captive_portal:
```
