#ifndef _VALVE_H_
#define VALVE_H_

#include <Arduino.h>

class Valve {
public:
  Valve(int open_cmd_pin, int close_cmd_pin, int opened_state_pin, int closed_state_pin);

  void checkState();
  
  void openValve();
  void closeValve();
  void stopValve();
  bool isOpen();
  bool isOpening();
  bool isClosed();
  bool isClosing();
  bool isRunning();
  int runningControl();

private:
  int open_cmd_pin;
  int close_cmd_pin;
  int open_state_pin;
  int closed_state_pin;
  char movement = 's'; // s - stop servo valve, o - open valve, c - close valve
  bool running = false;
  void init();
};

#endif // _VALVE_H_