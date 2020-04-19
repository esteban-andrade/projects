#include "vehicle.h"

Vehicle::Vehicle(){
  
}

Vehicle::Vehicle(std::string make, std::string model, std::string colour, int currentSpeed) {
  make_ = make;
  model_ = model;
  colour_ = colour;
  currentSpeed_ = currentSpeed;
}

std::string Vehicle::getMake(void) {
  return make_;
}

std::string Vehicle::getModel(void) {
  return model_;
}

std::string Vehicle::getColour(void) {
  return colour_;
}

int Vehicle::getCurrentSpeed(void) {
  return currentSpeed_;
}

void Vehicle::setMake(std::string make) {
  make_ = make;
}

void Vehicle::setModel(std::string model) {
  model_ = model;
}

void Vehicle::setColour(std::string colour) {
  colour_ = colour;
}

void Vehicle::setCurrentSpeed(int currentSpeed) {
  currentSpeed_  = currentSpeed;
}
