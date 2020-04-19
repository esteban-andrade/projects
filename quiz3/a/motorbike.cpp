#include "vehicle.h"
#include "motorbike.h"
#include <iostream>

Motorbike::Motorbike(){
  
}

Motorbike::Motorbike(std::string make, std::string model, std::string colour, int speed):
  Vehicle(make, model, colour, speed)
{
}

void Motorbike::details(){
  std::cout << "This Motorbike is a " << Vehicle::colour_ << " " << Vehicle::make_ << " "
  << Vehicle::model_ << " with "  << "It's current speed is "
  << Vehicle::currentSpeed_ << " km/h." << std::endl;
}
