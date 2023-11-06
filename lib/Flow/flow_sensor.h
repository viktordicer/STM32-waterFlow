#ifndef _FLOW_SENSOR_H_
#define _FLOW_SENSOR_H_

#include <Arduino.h>

class FlowSensor {
public:
  FlowSensor(int interrupt_pin);
  
  uint32_t  getRPM();
  void      clearRPM();
  void      flowCount();
  void      toLiters();
  float     getTotalVolume();
  void      setTotalVolume(float volume);
  float     getIncVolume();
  void      clearIncVolume();


private:
  int interrupt_pin;
  uint32_t rpm;
  float impulse_per_liter = 288.0;
  float inc_volume = 0.0; 
  float total_volume = 0.0;

  void init();

};


#endif // _FLOW_SENSOR_H_