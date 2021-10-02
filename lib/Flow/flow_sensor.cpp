
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
  return rpm;
}

// Set number of impulses to 0 - water flow sensor
void FlowSensor::clearRPM(){
  this->rpm = 0;
}

// Convert number of impulses to liters and clear rpm
double FlowSensor::toLiters(){
  double liters = rpm / impulse_per_liter;
  clearRPM();
  this->total_volume += liters;
  return liters;
}

//Get Total volume, in liters
double FlowSensor::getTotalVolume(){
  return total_volume;
} 
//Clear Total volume, in liters
void FlowSensor::clearTotalVolume(){
  this->total_volume = 0;
} 

// Set max volume of water in liters. When the value is reached, the servo valve closes automatically.
void FlowSensor::setMaxVolume(double max_volume){
  this->max_volume = max_volume;
}

// Get max volume of water in liters. When the value is reached, the servo valve closes automatically.
double FlowSensor::getMaxVolume(){
  return max_volume;
}