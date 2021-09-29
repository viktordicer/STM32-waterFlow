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


// Methodes
void callback(char* topic, byte* payload, unsigned int length);
void ethernetReset();
void mqttConnection();
void compareFlow();
void dataToChar();
void valveReadState();
void valveChangeState(int valve_num , int valve_state);
void ethernetTurnoff();

//------------------------- Program variables
//NEW Objects
EthernetClient ethClient;
PubSubClient mqttClient(MQTT_SERVER_IP, 1883, callback, ethClient);

//VARIABLES
uint32_t time = 0;
uint32_t last_time = 0;
uint32_t last_time_mqtt = 0;
uint32_t last_time_state = 0;

bool change_rpm = false;
bool mqtt_conn = false;

int connection_failed = 0;
char connection_failed_char[10];

//Interior
uint32_t rpmInt = 0;
uint32_t rpmInt_last = 0;
double lit_int = 0;
double lit_int_last = 0;
char litInterior[15];
double compareInterLit = 0;
char compareInterLit_msg[15];

//Exterior
uint32_t rpmExt = 0;
double lit_ext = 0;
double lit_ext_last = 0;
char litExterior[15];
double compareExterLit = 0;
char compareExterLit_msg[15];


// define valves pins
bool valveRun[4] = {0,0,0,0}; // 0- valve1 open, 1-valve1 close, 2-valve2 open, 3-valve2 close

Valve val1(V1_OPEN, V1_CLOSE, V1_OPENED, V1_CLOSED);
Valve val2(V2_OPEN, V2_CLOSE, V2_OPENED, V2_CLOSED);
FlowSensor flow_sensor1(INTERRUPT_INT_FLOW);
FlowSensor flow_sensor2(INTERRUPT_EXT_FLOW); 


void setup() {

  pinMode(GREEN_LED, OUTPUT); // green LED on Blackpill
  digitalWrite(GREEN_LED, HIGH); // 

  Ethernet.begin(MAC);
  delay(1500);
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  mqttClient.setCallback(callback);
  
}

// ------------------ LOOP ---------------------------------------------------------
void loop() {
  // If first run open all valves
  Ethernet.maintain();
  time = millis();

  // *------------------------Solve liters-----------------------------------------
  if(time-last_time > DELAY){
    if(flow_sensor1.getRPM() > 0){
      lit_int += flow_sensor1.toLiters();
      if(flow_sensor2.getRPM() > 0){
        lit_ext += flow_sensor2.toLiters();
      }
      compareFlow();
      change_rpm = true;
    }else{
      change_rpm = false;
    }
    last_time = time;
  }

// * -------------------- MQTT connection and send data---------------------------------
  if(time-last_time_mqtt > MQTT_CHECK_CONNECTION){
  //CHECK MQTT CONNECTION IS ALIVE
    if(mqtt_conn == false){
      ethernetReset();
    }
    if(!mqttClient.connected()){
        Serial.println("MQTT connection is killed.");
        mqttConnection();
      }else{
        Serial.println("MQTT connection is still alive.");
      }

    if(mqtt_conn==true){

      Serial.print("Total LAN connection lost: ");
      Serial.println(connection_failed);
      dataToChar();
      mqttClient.publish(TECHNICAL_CONNECTION_FAILED, connection_failed_char);
      //check changing flow
      Serial.println(lit_int);
      if(change_rpm == false && mqtt_conn == true) {
        if(lit_int != lit_int_last){
          Serial.print("Liters INT: ");
          Serial.println(litInterior);
          
          Serial.print("Liters INT difference: ");
          Serial.println(compareInterLit_msg);
          mqttClient.publish(LIT_INT_TOPIC, compareInterLit_msg);
          mqttClient.publish(LIT_INT_SUM_TOPIC, litInterior);
        }

        if(lit_ext != lit_ext_last){
          Serial.print("Liters EXT: ");
          Serial.println(litExterior);
          Serial.print("Liters EXT difference: ");
          Serial.println(compareExterLit_msg);
          mqttClient.publish(LIT_EXT_TOPIC, compareExterLit_msg);
          mqttClient.publish(LIT_EXT_SUM_TOPIC, litExterior);
        }

        lit_int_last = lit_int;
        lit_ext_last = lit_ext;
      }
      last_time_mqtt = time;
    }
  }
  mqttClient.loop();
  valveReadState();
}

// *---------------------------------------------------SENDING DATA -------------------------------------------------------


// *--------------------- MQTT CONNECTION --------------------
void mqttConnection() {
  // ADD MQTT connection call
  // Reset ethernet adapter
  //ethernetReset();
  // setup mqtt client
  int connectionAtempt = 1;
  if(!mqttClient.connected()){
    while (!mqttClient.connected()) {
      //after 2 atempts end trying connection
      if(connectionAtempt > 3 ){
        digitalWrite(PC13, LOW);
        mqttClient.disconnect();
        delay(300);
        Serial.println("Ethernet adapter has turned off");
        ethernetReset();
        connection_failed ++;
        mqtt_conn = false;
        Serial.println("Can't connect to the mqtt.");
        return;
      }
      Serial.println("Attempting MQTT connection...");
      // Attempt to connect

      //Unique MQTT client ID
      /*String clientId = "Technical_room-";
      clientId += String(random(0xffff), HEX);*/
      // Mqtt connection
      if (mqttClient.connect(MQTT_CLIENT_ID, USERNAME, PASSWORD )) {
        Serial.println("MQTT connected");
        mqttClient.subscribe(SUBSCRIBE_TOPIC);
        mqttClient.subscribe(SUBSCRIBE_TOPIC_INT);
        mqttClient.subscribe(SUBSCRIBE_TOPIC_EXT);
        digitalWrite(GREEN_LED, HIGH);
        mqtt_conn = true;
      } else {
        Serial.println("failed, rc=");
        Serial.println(mqttClient.state());
        Serial.println(" try again in 2 seconds");
        delay(1000);
        Serial.print("Connection atempt: "); 
        Serial.println(connectionAtempt);
        connectionAtempt ++;
        digitalWrite(PC13, LOW);
        //ethernetReset();
      }
    }
  }
  delay(200);
  Serial.println("Connected to the mqtt");
}

// *----------------------RESET ETHERNET---------------
void ethernetReset() {
  //Define reset pin for W5500
  Serial.println("Reseting ethernet adapter");
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN,HIGH);
  delay(100);
  pinMode(RESET_PIN, INPUT);
  delay(100);
  Serial.println("Initialize ethernet");
  Ethernet.begin(MAC);

  //delay with ethernet maintain
  for (int i = 0; i <= 10; i++) {
    delay(100);
    Ethernet.maintain();
  }
  mqttClient.setServer(MQTT_SERVER_IP,1883);
  mqttClient.setCallback(callback);
}

void ethernetTurnoff(){
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  Serial.println("Ethernet adapter has turned off");
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
    if(msg == 1) {
      //mqttClient.publish(VALVE1_STATE_TOPIC, "opened");
      valveChangeState(1,1); //open internal valve
    // Close valve internal
    } else if (msg == 2) {
      //mqttClient.publish(VALVE1_STATE_TOPIC, "closed");
      valveChangeState(1,0); //close internal valve
    // Open valve external
    } else if (msg == 3) {
      //mqttClient.publish(VALVE2_STATE_TOPIC, "opened");
    valveChangeState(2,1); //open external valve

    // Close valve external
    } else if (msg == 4) {
      //mqttClient.publish(VALVE2_STATE_TOPIC, "closed");
      valveChangeState(2,0); //close external valve
    }
  }else if(strcmp(topic,SUBSCRIBE_TOPIC_INT)==0){
    payload[length] = '\0';
    String s = String((char*)payload);
    max_internal_liters = s.toDouble();
    Serial.println(max_internal_liters);
    
    mqttClient.publish(LIT_INT_MAX_TOPIC, p, length);

  
  }else if(strcmp(topic,SUBSCRIBE_TOPIC_EXT)==0){
    payload[length] = '\0';
    String s = String((char*)payload);
    max_external_liters = s.toDouble();
    Serial.println(max_external_liters);
    mqttClient.publish(LIT_EXT_MAX_TOPIC, p, length);
  }

  // Free the memory
  free(p);
  
}


// check liters
void compareFlow() {
  compareInterLit = lit_int - lit_int_last - lit_ext + lit_ext_last;
  compareExterLit = lit_ext - lit_ext_last;
  if(compareInterLit > max_internal_liters){
    Serial.println(compareInterLit);
    valveChangeState(1,0); //close internal valve
    mqttClient.publish(SUBSCRIBE_TOPIC, "2");
  }
  if(compareExterLit > max_external_liters){
    // close valve2
    Serial.println(compareExterLit);
    valveChangeState(2,0); //close external valve 
    mqttClient.publish(SUBSCRIBE_TOPIC, "4");

  }
}


// read valve state
void valveReadState() {
  for(int i =0; i<4; i++){
    if(valveRun[i] == 1 && digitalRead(valvePosition[i]) == 0){
      digitalWrite(valveSet[i], LOW);
      valveRun[i] = 0;
      if(i==0){
        mqttClient.publish(VALVE1_STATE_TOPIC, "opened");
        //mqttClient.publish(SUBSCRIBE_TOPIC, "1");
        mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      } else if(i==1){
        mqttClient.publish(VALVE1_STATE_TOPIC, "closed");
        //mqttClient.publish(SUBSCRIBE_TOPIC, "2");
        mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      } else if(i==2){
        mqttClient.publish(VALVE2_STATE_TOPIC, "opened");
        //mqttClient.publish(SUBSCRIBE_TOPIC, "3");
        mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      } else if(i==3){
        mqttClient.publish(VALVE2_STATE_TOPIC, "closed");
        //mqttClient.publish(SUBSCRIBE_TOPIC, "4");
        mqttClient.publish(VALVE_ERROR_TOPIC, "no error");
      }
    }else if((valveRun[i] == 1) && (time-last_time_state > VALVE_PROTECTION_PERIOD)) {
      digitalWrite(valveSet[i], LOW);
      valveRun[i] = 0;
      mqttClient.publish(VALVE_ERROR_TOPIC, "valve error");
    }
  }
}

/**
 * @brief Change setvo valve state - open or close 
 * @param  valve_num : number of servo valve 1 for internal 2 for external
 * @param  valve_state : 1 - open valve , 0 - close valve
 */
void valveChangeState(int valve_num , int valve_state){
  last_time_state = time;
  //open valve1
  if((valve_num == 1) && valve_state == 1 ){
    digitalWrite(valveSet[1],LOW);
    delay(50);
    digitalWrite(valveSet[0],HIGH);
    valveRun[1]=0;
    valveRun[0]=1;
    Serial.println("Opening Internal valve");
  }
  //close valve1
  if((valve_num == 1) && valve_state == 0 ){
    digitalWrite(valveSet[0],LOW);
    delay(50);
    digitalWrite(valveSet[1],HIGH);
    valveRun[0]=0;
    valveRun[1]=1;
    Serial.println("Closing Internal valve");
  }

  //open valve2
  if((valve_num == 2) && valve_state == 1 ){
    digitalWrite(valveSet[3],LOW);
    delay(50);
    digitalWrite(valveSet[2],HIGH);
    valveRun[3]=0;
    valveRun[2]=1;
    Serial.println("Opening Internal valve");

  }
  //close valve2
  if((valve_num == 2) && valve_state == 0 ){
    digitalWrite(valveSet[2],LOW);
    delay(50);
    digitalWrite(valveSet[3],HIGH);
    valveRun[2]=0;
    valveRun[3]=1;
    Serial.println("Closing External valve");
  }
}


// *------------------------------------------DATA TO CHAR -------------------------------------------------------
void dataToChar(){
  dtostrf(lit_int,8,2,litInterior);
  dtostrf(lit_ext,8,2,litExterior);
  dtostrf(compareInterLit,6,2,compareInterLit_msg);
  dtostrf(compareExterLit,6,2,compareExterLit_msg);
  sprintf(connection_failed_char, "%i", connection_failed);
}

