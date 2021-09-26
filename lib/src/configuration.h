//-------------HARDWARE VARIABLES
//ETHERNET
#define RESET_PIN               PA0 // Reset of ENC28J60

//Water flow sensors
#define INTERRUPT_INT_FLOW      PB9 // internal circuit flow sensor
#define INTERRUPT_EXT_FLOW      PB8 // external circuit flow sensor

//VALVE commands
#define V1_OPEN                 PB7   // internal valve open command
#define V1_CLOSE                PB6   // internal valve close command
#define V2_OPEN                 PB3   // external valve open command
#define V2_CLOSE                PA15  // external valve close command

// VALVE states 
#define V1_OPENED               PB12   // internal valve opened state
#define V1_CLOSED               PB13   // internal valve closed state
#define V2_OPENED               PA9  // external valve opened state
#define V2_CLOSED               PA8  // external valve closed state

//ETHERNET ADDRESS
int IP_ADDRESS[4]=        {192, 168, 0 , 170};
uint8_t MAC[6] =          {0x02, 0xA1, 0xA5, 0x03, 0x04, 0x05};

//MQTT
#define MQTT_SERVER_IP           "192.168.0.107"   // IP address of MQTT broker
#define MQTT_CLIENT_ID           "Technical_room_water" // require ID for MQTT client
#define USERNAME                 "viktor"         // username MQTT broker
#define PASSWORD                 "viktor"         // password MQTT broker
#define MQTT_CHECK_CONNECTION    20000            // check connection is alive in ms 
long    total_mqtt_connection =  0;                // NUmbet of total connection call
#define MQTT_SENDING_DEALY       50 // mqtt delay between publish

//TOPICS
// waterflow sensor liter topic
#define LIT_INT_TOPIC                    "sensor/technical/lit_inter"
#define LIT_EXT_TOPIC                    "sensor/technical/lit_exter"
#define LIT_INT_SUM_TOPIC                "sensor/technical/lit_inter_sum"
#define LIT_EXT_SUM_TOPIC                "sensor/technical/lit_exter_sum"
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
#define DELAY              10000 // delay in ms
#define MAX_RPM_COUNTS     80000 //reset rpm counts after reach max
//valve states array

// conversion variables
#define LITER_CONVERSION    288.0 // conversion value 1L => 288 impulses


//Failure variables
int     lan_connection_lost =   0;// Number of MQTT connection lost 
#define VALVE_PROTECTION_PERIOD 10000 // automatically turn off servo valve when no feedback from internal switch




