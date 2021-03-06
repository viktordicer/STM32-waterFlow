
#include "valve.h"

Valve::Valve(int open_cmd_pin, int close_cmd_pin, int open_state_pin, int closed_state_pin) {

  this->open_cmd_pin = open_cmd_pin;
  this-> close_cmd_pin = close_cmd_pin;
  this->open_state_pin = open_state_pin;
  this->closed_state_pin = closed_state_pin;
  init();
}

void Valve::init(){
  pinMode(open_cmd_pin, OUTPUT);
  pinMode(close_cmd_pin, OUTPUT);
  pinMode(open_state_pin, INPUT);
  pinMode(closed_state_pin, INPUT);
  
  digitalWrite(open_cmd_pin, LOW);
  digitalWrite(close_cmd_pin, LOW);
}

void Valve::checkState(){
  if(movement == 'o'){
    if(isOpen()){
        stopValve();
    }
  }else if(movement == 'c'){
    if(isClosed()){
        stopValve();
    }
  }
}

// Open the servo valve
void Valve::openValve(){
  this->movement = 'o';
  this->running = true;
  this->state = 1;
  digitalWrite(close_cmd_pin, LOW);
  delay(100);
  digitalWrite(open_cmd_pin, HIGH);
}

//Close the servo valve
void Valve::closeValve(){
  this->movement = 'c';
  this->running = true;
  this->state = 0;
  digitalWrite(open_cmd_pin, LOW);
  delay(100);
  digitalWrite(close_cmd_pin, HIGH);
}

// Stop the servo valve
void Valve::stopValve(){
  this->movement = 's';
  this->running = false;
  digitalWrite(close_cmd_pin, LOW);
  digitalWrite(open_cmd_pin, LOW);
}

// Read opened pin - if is LOW return true
bool Valve::isOpen(){
  if(digitalRead(open_state_pin) == LOW) {
    return true;
  }
  return false;
}

//Read if servo valve is opening
bool Valve::isOpening(){
  if(isRunning()){
    if(movement == 'o'){
      return true;
    }
  }
  return false;
}

// Read closed pin - if is LOW return true
bool Valve::isClosed(){
  if(digitalRead(closed_state_pin) == LOW) {
    return true;
  }
  return false;
}

//Read if servo valve is closing
bool Valve::isClosing(){
  if(isRunning()){
    if(movement == 'c'){
      return true;
    }
  }
  return false;
}

//Check if servo valve is running
bool Valve::isRunning(){
  return running;
}

// Get state of valve 0- closed 1-open
int Valve::getState(){
  return state;
}

// Check is servo valve is running and in the end position, then turn off servo. 
//1 => valve is open , 0 => valve is closed , 2 - servo not running
int Valve::runningControl(){
  if(isRunning()){
    switch(movement){
      case 'o':
        if(isOpen()){
          stopValve();
          return 1;
        }
      break;
      case 'c':
        if(isClosed()){
          stopValve();
          return 0;
        }
      break;
    }
  }
  return 2; // not running
}