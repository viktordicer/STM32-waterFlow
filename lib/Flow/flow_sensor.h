#ifndef _FLOW_SENSOR_H_
#define _FLOW_SENSOR_H_

#include <Arduino.h>

class FlowSensor {
public:
  FlowSensor(int interrupt_pin);
  
  uint32_t getRPM();
  void clearRPM();
  void flowCount();
  void toLiters();
  double getTotalVolume();
  void clearTotalVolume();


private:
  int interrupt_pin;
  u_int32_t rpm;

  double impulse_per_liter = 288.0;
  double total_volume = 0.0;

  void init();

};


#endif // _FLOW_SENSOR_H_