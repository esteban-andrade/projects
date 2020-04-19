#include "vehicle.h"
#include "motorbike.h"
#include <iostream>

Motorbike::Motorbike(std::string make, std::string model, std::string colour, int speed):
  Vehicle(make, model, colour, speed)
{
}

void Motorbike::accelerate(void) {
  Vehicle::currentSpeed_++;
}

void Motorbike::decelerate(void) {
  Vehicle::currentSpeed_--;
}

void Motorbike::details (void) {
  std::cout << "This motorbike is a " << Vehicle::colour_ << " " << Vehicle::make_ << " "
  << Vehicle::model_ << ". It's current speed is "
  << Vehicle::currentSpeed_ << " km/h." << std::endl; 
  }
