#ifndef _VALVE_H_
#define VALVE_H_

#include <Arduino.h>
class Valve {
public:
  Valve(uint8_t open_cmd_pin, uint8_t close_cmd_pin, uint8_t opened_state_pin, uint8_t closed_state_pin);

  void checkState();
  
  void openValve();
  void closeValve();
  void stopValve();
  bool isOpen();
  bool isClosed();
  bool isRunning();

private:
  void init();
  uint8_t open_cmd_pin;
  uint8_t close_cmd_pin;
  uint8_t open_state_pin;
  uint8_t closed_state_pin;
  char movement = 's'; // s - stop servo valve, o - open valve, c - close valve
  bool running = false;
};

#endif // _VALVE_H_