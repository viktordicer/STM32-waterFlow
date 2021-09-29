
#include "flow_sensor.h"

FlowSensor::FlowSensor(int interrupt_pin) {
  this->interrupt_pin = interrupt_pin;
  init();
}

void FlowSensor::init() {
  pinMode(interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), flowCount, RISING);
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
  return liters;
}

// Max number stored of flow sensor impulses
void FlowSensor::setMaxRpmCount(uint32_t max_rpm_count){
  this->max_rpm_count = max_rpm_count;
}