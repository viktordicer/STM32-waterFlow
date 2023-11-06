#include "flow_sensor.h"

FlowSensor::FlowSensor(int interrupt_pin) {
  this->interrupt_pin = interrupt_pin;
  init();
}

void FlowSensor::init() {
  pinMode(interrupt_pin, INPUT_PULLUP);
}

void FlowSensor::flowCount() {
  this->rpm ++;
}

// Get the number of impulses - water flow sensor
uint32_t FlowSensor::getRPM(){
  return this->rpm;
}

// Set number of impulses to 0 - water flow sensor
void FlowSensor::clearRPM(){
  this->rpm = 0;
}

// Convert number of impulses to liters and clear rpm
void FlowSensor::toLiters(){
  float liters = this->rpm / impulse_per_liter;
  clearRPM();
  this->inc_volume += liters;
  this->total_volume += liters/1000;
}

//Get Increment volume, in liters
float FlowSensor::getIncVolume(){
  return this->inc_volume;
} 

//Clear Increment volume, in liters
void FlowSensor::clearIncVolume(){
  this->inc_volume = 0;
} 

//Get Total volume, in m3
float FlowSensor::getTotalVolume(){
  return this->total_volume;
}

//Set Total volume, in m3
void FlowSensor::setTotalVolume(float volume){
  this->total_volume = volume;
}


