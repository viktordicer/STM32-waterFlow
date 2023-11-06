
#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

//-------------HARDWARE VARIABLES
// BLACKPILL
#define GREEN_LED               PC13 // Green LED on BlackPill - when not able to connect to MQTT brocker
//ETHERNET
#define RESET_PIN               PA0 // Reset of ENC28J60

uint8_t MAC[6] =          {0x02, 0xA1, 0xA5, 0x03, 0x04, 0x05};

//Water flow sensors
#define INTERRUPT_INT_FLOW      PB9 // internal circuit flow sensor
#define INTERRUPT_EXT_FLOW      PB8 // external circuit flow sensor

//VALVE 1
#define V1_OPEN                 PB5   // internal valve open command
#define V1_CLOSE                PB4   // internal valve close command
#define V1_OPENED               PB12   // internal valve opened state
#define V1_CLOSED               PB13   // internal valve closed state

// VALVE 2 
#define V2_OPEN                 PB3   // external valve open command
#define V2_CLOSE                PA15  // external valve close command
#define V2_OPENED               PA9  // external valve opened state
#define V2_CLOSED               PA8  // external valve closed state


//MQTT
#define MQTT_SERVER_IP           "192.168.0.107"   // IP address of MQTT broker
#define MQTT_CLIENT_ID           "Technical_room_water" // require ID for MQTT client
#define USERNAME                 "viktor"         // username MQTT broker
#define PASSWORD                 "viktor"         // password MQTT broker
#define MQTT_CHECK_CONNECTION    20000            // check connection is alive in ms 

//TOPICS
#define HA_TOPIC                         "homeassistant/status"
#define STATUS_TOPIC                     "sensor/technical/status"

// waterflow sensor liter topic
#define LIT_INT_TOPIC                    "sensor/technical/lit_inter"
#define LIT_INT_TOTAL_TOPIC              "sensor/technical/lit_inter_total"
#define LIT_EXT_TOPIC                    "sensor/technical/lit_exter"
#define LIT_EXT_TOTAL_TOPIC              "sensor/technical/lit_exter_total"
// technical valve state
#define VALVE1_STATE_TOPIC               "sensor/technical/valve_inter"
#define VALVE2_STATE_TOPIC               "sensor/technical/valve_exter"
#define VALVE_ERROR_TOPIC                "sensor/technical/valve_error"
// max liters confirm topics
#define LIT_INT_MAX_TOPIC                "sensor/technical/internal_max"
#define LIT_EXT_MAX_TOPIC                "sensor/technical/external_max"
// command fow open and close servo valves
#define SUBSCRIBE_TOPIC                  "command/technical/waterflow"
#define SUBSCRIBE_TOPIC_INT              "sensor/technical/set_internal_max"
#define SUBSCRIBE_TOPIC_EXT              "sensor/technical/set_external_max"
// failed
#define TECHNICAL_CONNECTION_FAILED      "sensor/technical/connection_failed"


//VARIABLES
#define DELAY              8000 // delay in ms
double max_internal_volume = 30; // max volume for internal servo valve
double max_external_volume = 200; // max volume for internal servo valve

//Failure variables
int     lan_connection_lost =   0;// Number of MQTT connection lost 
#define VALVE_PROTECTION_PERIOD 12000 // automatically turn off servo valve when no feedback from internal switch

#endif // _CONFIGURATION_H_


