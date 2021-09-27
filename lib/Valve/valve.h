#ifndef _VALVE_H_
#define VALVE_H_

#include <Arduino.h>
class Valve {
public:
    /**
     * @brief Construct a new Valve object
     * 
     * @param open_cmd_pin 
     * @param close_cmd_pin 
     * @param opened_state_pin 
     * @param closed_state_pin 
     */
    Valve(byte open_cmd_pin, byte close_cmd_pin, byte opened_state_pin, byte closed_state_pin);

    void get_state();
    void set_state();

private:
    void init();
    byte open_cmd_pin;
    byte close_cmd_pin;
    byte opened_state_pin;
    byte closed_state_pin

};

#endif // _VALVE_H_