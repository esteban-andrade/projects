#include <iostream>
#include "vehicle.h"
#include "car.h"

Car::Car(std::string make, std::string model, std::string colour, int speed, unsigned int numOfAirbags):
  Vehicle(make, model, colour, speed)
{
  numOfAirbags_ = numOfAirbags;

}

void Car::accelerate(void) {
  Vehicle::currentSpeed_++;
}

void Car::decelerate(void) {
  Vehicle::currentSpeed_--;
}

void Car::details (void) {
  std::cout << "This car is a " << Vehicle::colour_ << " " << Vehicle::make_ << " "
  << Vehicle::model_ << " with " << numOfAirbags_ << " airbags. It's current speed is "
  << Vehicle::currentSpeed_ << " km/h." << std::endl;
}
