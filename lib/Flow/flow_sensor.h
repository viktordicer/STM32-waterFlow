#ifndef _FLOW_SENSOR_H_
#define _FLOW_SENSOR_H_

#include <Arduino.h>

class FlowSensor {
public:
  FlowSensor(int interrupt_pin);
  
  uint32_t getRPM();
  void clearRPM();
  double toLiters();
  void setMaxLiter(double max_liters);
  void setMaxRpmCount(uint32_t max_rpm_count);

private:

  int interrupt_pin;
  u_int32_t rpm;

  double impulse_per_liter = 288.0;
  uint32_t max_rpm_count = 80000;
  double max_liters = 20;

  void init();
  void flowCount();

};


#endif // _FLOW_SENSOR_H_