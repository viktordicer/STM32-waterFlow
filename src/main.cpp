/********************************
  W5500 TO STM32 TYPICAL WIRING
    VCC ----> 5V
    GND ----> GND
    CS  ----> PA4
    SCK ----> PA5
    SO  ----> PA6
    SI  ----> PA7
    RST ----> PA0

  H-bridge driver
  INT1  ----> PB7      //MOTOR A Yellow (YELLOW)
  INT2  ----> PB6      //MOTOR A Blue (GREEN)
  INT3  ----> PB3      //MOTOR B Yellow (YELLOW)
  INT4  ----> PA15     //MOTOR B Blue (GREEN)

  Flow sensor for INTERNAL circuit
    int0 ----> PB9

  Flow sensor for EXTERNAL circuit
    int0 ----> PB8

  Servo valve INTERNAL
    open_valve    ----> PB7
    close_valve   ----> PB6
    opened_state  ----> PB12   Green (GREY)
    closed_state  ----> PB13  Red (WHITE)
    GND           ----> GND   Black (BROWN)

  Servo valve EXTERNAL
    open_valve    ----> PB3
    close_valve   ----> PA15
    opened_state  ----> PA9   Green
    closed_state  ----> PA8   Red


  MQTT incoming comands from Home assistant
    Topic - "command/technical/waterflow"
    msgs:
              1 --> open internal valve
              2 --> close internal valve
              3 --> openb external valve
              4 --> close external valve

    Topic - "sensor/technical/valve_inter" - state for internal valve
    msgs:
              for opened state  --> "opened"
              for closed state  --> "closed"

    Topic - "sensor/technical/valve_exter" - state for external valve
    msgs:
              for opened state  --> "opened"
              for closed state  --> "closed"

*********************************/

// LIBRARY
#include <Arduino.h>
#include <Ethernet.h>  //ETHERNET LIBRARY
#include <PubSubClient.h> //MQTT LIBRARY
#include "configuration.h"
#include "valve.h"
#include "flow_sensor.h"

// DEBUG MODE DEFINE - in debug mode serial print is on
//#define DEBUG


// Methodes
void setup();
void loop();
void callback(char* topic, byte* payload, unsigned int length);
void ethernetReset();
void mqttConnection();
void compareFlow();
void dataToChar();
void convertToLiters();
void valveState();
void sendData();
void serialPrint(String message);
void countINT();
void countEXT();
void valvePosition();

//------------------------- Program variables
//NEW Objects
EthernetClient ethClient;
PubSubClient mqttClient(MQTT_SERVER_IP, 1883, callback, ethClient);

Valve val1(V1_OPEN, V1_CLOSE, V1_OPENED, V1_CLOSED); //servo valve - internal
Valve val2(V2_OPEN, V2_CLOSE, V2_OPENED, V2_CLOSED); //servo valve - external
FlowSensor flow_sensor1(INTERRUPT_INT_FLOW); // flow sensor internal
FlowSensor flow_sensor2(INTERRUPT_EXT_FLOW); // flow sensor external

//VARIABLES
uint32_t time = 0;
uint32_t last_time = 0;
uint32_t last_time_mqtt = 0;

bool change_rpm = false;
bool mqtt_conn = false;
bool first_run = true;

int connection_failed = 0;
int connection_failed_last = 0;
char connection_failed_char[10];

//Interior
double total_inter_volume = 0;
char total_inter_volume_msg[15];

//Exterior
double total_exter_volume = 0;
char total_exter_volume_msg[15];

void setup() {
  #ifdef DEBUG
    Serial.begin(57600);
    delay(4000);
    Serial.println("STM32 starting...");
    Serial.println("Debug mode");
  #endif 


  pinMode(GREEN_LED, OUTPUT); // green LED on Blackpill
  digitalWrite(GREEN_LED, HIGH); // 
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_INT_FLOW), countINT, RISING);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_EXT_FLOW), countEXT, RISING);

  Ethernet.begin(MAC);
  delay(1500);
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  mqttClient.setCallback(callback);
  
}

// ------------------ LOOP ---------------------------------------------------------
void loop() {
  Ethernet.maintain();
  time = millis();

  if(first_run){
    mqttConnection();
    mqttClient.publish(TECHNICAL_CONNECTION_FAILED, "0");
    //valvePosition();
    if(!val1.isOpen()){
      val1.openValve();
    }
    first_run = false;
  }

  // *------------------------Volume-----------------------------------------
  if(time-last_time > DELAY){
    convertToLiters();
    last_time = time;
  }

  if(time-last_time_mqtt > MQTT_CHECK_CONNECTION){
  //CHECK MQTT CONNECTION IS ALIVE
    if(mqtt_conn == false){
      ethernetReset();
    }
    if(!mqttClient.connected()){
        serialPrint("MQTT connection is killed.");
        mqttConnection();
      }else{
        serialPrint("MQTT connection is still alive.");
      } 
    sendData(); //send data to MQTT broker   
  }
  mqttClient.loop();
  delay(100);
  valveState();
}


// *--------------------- MQTT CONNECTION --------------------
void mqttConnection() {
  int connectionAtempt = 1;
  if(!mqttClient.connected()){
    while (!mqttClient.connected()) {
      //after 2 atempts end trying connection
      if(connectionAtempt > 3 ){
        digitalWrite(GREEN_LED, LOW);
        mqttClient.disconnect();
        delay(300);
        ethernetReset();
        connection_failed ++;
        mqtt_conn = false;
        serialPrint("Can't connect to the mqtt.");
        return;
      }
      serialPrint("Attempting MQTT connection...");

      if (mqttClient.connect(MQTT_CLIENT_ID, USERNAME, PASSWORD )) {
        serialPrint("MQTT connected");
        mqttClient.subscribe(SUBSCRIBE_TOPIC);
        mqttClient.subscribe(SUBSCRIBE_TOPIC_INT);
        mqttClient.subscribe(SUBSCRIBE_TOPIC_EXT);
        digitalWrite(GREEN_LED, HIGH);
        mqtt_conn = true;
      } else {
        serialPrint(" try again in 2 seconds");
        delay(1000);
        connectionAtempt ++;
        digitalWrite(GREEN_LED, LOW);
        //ethernetReset();
      }
    }
  }
  delay(200);
}


// *----------------------RESET ETHERNET---------------
void ethernetReset() {
  //Define reset pin for W5500
  serialPrint("Reseting ethernet adapter");
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN,HIGH);
  delay(100);
  pinMode(RESET_PIN, INPUT);
  delay(100);
  serialPrint("Initialize ethernet");
  Ethernet.begin(MAC);

  //delay with ethernet maintain
  for (int i = 0; i <= 10; i++) {
    delay(100);
    Ethernet.maintain();
  }
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  mqttClient.setCallback(callback);
}

// Send data to mqtt brocker
void sendData(){
  if(mqtt_conn==true){
      serialPrint("mqtt sending data");
      dataToChar();
      if(connection_failed != connection_failed_last){
        mqttClient.publish(TECHNICAL_CONNECTION_FAILED, connection_failed_char);
        connection_failed_last=connection_failed;
      }
      //check changing flow
      if(change_rpm == false && mqtt_conn == true) {
        if(total_inter_volume > 0){
          mqttClient.publish(LIT_INT_TOPIC, total_inter_volume_msg);
          total_inter_volume = 0;
        }

        if(total_exter_volume > 0){
          mqttClient.publish(LIT_EXT_TOPIC, total_exter_volume_msg);
          total_exter_volume =  0;
        }
      }
      last_time_mqtt = time;
    }
}


// *MQTT CALLBACK 
void callback(char* topic, byte* payload, unsigned int length) {

  byte* p = (byte*)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  
  if(strcmp(topic,SUBSCRIBE_TOPIC)==0){
    int msg;
    for(int i=0; i<length; i++){
    msg = (uint16_t)payload[i] -48;
    }
    // Open valve internal
    serialPrint("callback valve command");
    serialPrint(String(msg));
    switch (msg)
    {
    
    case 1:
      serialPrint("Open internal valve");
      serialPrint(String(val1.isOpening()));
      serialPrint(String(val1.isOpen()));
      if(!val1.isOpening() && !val1.isOpen()){
        val1.openValve(); //open internal valve
      }
      break;
    
    case 2:
      serialPrint("Close internal valve");
      serialPrint(String(val1.isClosing()));
      serialPrint(String(val1.isClosed()));
      if (!val1.isClosing() && !val1.isClosed()) {
        val1.closeValve(); //close internal valve
      }
      break;
    case 3:
      serialPrint("Open external valve");
      serialPrint(String(val2.isOpening()));
      serialPrint(String(val2.isOpen()));
      if(!val2.isOpening() && !val2.isOpen()) {
        serialPrint("Open external command");
        val2.openValve(); //open external valve
        serialPrint(String(val2.isRunning()));
      }
      break;
    case 4:
      serialPrint("Close external valve");
      serialPrint(String(val2.isClosing()));
      serialPrint(String(val2.isClosed()));
      if (!val2.isClosing() && !val2.isClosed()) {
        val2.closeValve(); //close external valve
      }
      break;
    }
  }else if(strcmp(topic,SUBSCRIBE_TOPIC_INT)==0){
    payload[length] = '\0';
    String s = String((char*)payload);
    max_internal_volume = s.toDouble();
    serialPrint(s);
    mqttClient.publish(LIT_INT_MAX_TOPIC, p, length);
  
  }else if(strcmp(topic,SUBSCRIBE_TOPIC_EXT)==0){
    payload[length] = '\0';
    String s = String((char*)payload);
    max_external_volume = s.toDouble();
    serialPrint(s);
    mqttClient.publish(LIT_EXT_MAX_TOPIC, p, length);
  }else if(strcmp(topic,SUBSCRIBE_TOPIC_STATE)==0){
    valvePosition();
  }
  free(p);
}

// RMP of flow - internal flow sensor, attachInterrupt calback function
void countINT(){
  flow_sensor1.flowCount();
}

// RMP of flow - external flow sensor, attachInterrupt calback function
void countEXT(){
  flow_sensor2.flowCount(); 
}

// *------------------------------------------convert impulses to liters -------------------------------------------------------
void convertToLiters() {
  serialPrint(String(flow_sensor1.getRPM()));
  if(flow_sensor1.getRPM() > 0){
      flow_sensor1.toLiters();
      if(flow_sensor2.getRPM() > 0){
        flow_sensor2.toLiters();
      }
      compareFlow();
      change_rpm = true;
    }else{
      change_rpm = false;
      flow_sensor1.clearTotalVolume();
      flow_sensor2.clearTotalVolume();
    }
}

// check liters
void compareFlow() {
  total_exter_volume = flow_sensor2.getTotalVolume();
  total_inter_volume = flow_sensor1.getTotalVolume() - total_exter_volume;
  if(total_inter_volume > max_internal_volume){
    val1.closeValve(); //close internal valve
    mqttClient.publish(SUBSCRIBE_TOPIC, "2");
  }
  if(total_exter_volume > max_external_volume){
    // close valve2
    val2.closeValve(); //close external valve 
    mqttClient.publish(SUBSCRIBE_TOPIC, "4");
  }
}

//Check both servo valves if is ruuning and if in in the ending position
void valveState(){
  // Check if servo valves is running
  int state_val1 = val1.runningControl();
  int state_val2 = val2.runningControl();

  switch(state_val1){
    case 0:
      mqttClient.publish(VALVE1_STATE_TOPIC, "closed");
      mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      serialPrint("Internal closed");
      break;
    case 1:
      mqttClient.publish(VALVE1_STATE_TOPIC, "open");
      mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      serialPrint("Internal epened");
      break;
  }
  switch(state_val2){
    case 0:
      mqttClient.publish(VALVE2_STATE_TOPIC, "closed");
      mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      serialPrint("External closed");
      break;
    case 1:
      mqttClient.publish(VALVE2_STATE_TOPIC, "open");
      mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      serialPrint("External opened");
      break;
  }
}

//Get both servo valves position, send to MQTT brocker
void valvePosition(){
  if(val1.isOpen()){
    mqttClient.publish(VALVE1_STATE_TOPIC, "open");
    mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
  }else if(val1.isClosed()){      
    mqttClient.publish(VALVE1_STATE_TOPIC, "closed");
    mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
  }
  if(val2.isOpen()){
      mqttClient.publish(VALVE2_STATE_TOPIC, "open");
      mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
  }else if(val2.isClosed()){
      mqttClient.publish(VALVE2_STATE_TOPIC, "closed");
      mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
  }
}

// Convert variables to char - MQTT messages
void dataToChar(){
  dtostrf(total_inter_volume,6,2,total_inter_volume_msg);
  dtostrf(total_exter_volume,6,2,total_exter_volume_msg);
  sprintf(connection_failed_char, "%i", connection_failed);
}

// Serial print if debug mode is on
void serialPrint(String message){
  #ifdef DEBUG
    Serial.println(message);
  #endif
}