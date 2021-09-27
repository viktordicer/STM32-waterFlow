
#include "valve.h"

Valve::Valve(byte open_cmd_pin, byte close_cmd_pin, byte opened_state_pin, byte closed_state_pin) {

    this->open_cmd_pin = open_cmd_pin;
    this->close_cdm_pin = close_cmd_pin;
    this->opened_state_pin = opened_state_pin;
    this->closed_state_pin = closed_state_pin;
    init();
}

Valve::init(){
    pinMode(open_cmd_pin, OUTPUT);
    pinMode(close_cmd_pin, OUTPUT);
    pinMode(opened_state_pin, INPUT_PULLUP);
    pinMode(closed_state_pin, INPUT_PULLUP);
    
}

Valve::get_state(){
    
}