#ifndef CAR_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define CAR_H

#include <string>
#include "vehicle.h"

class Car: public Vehicle {
public:
  Car(std::string, std::string, std::string, int, unsigned int);

  
  void accelerate(void); //Increment the currentSpeed of the vehicle by one
  void decelerate(void); //Decrement the currentSpeed of the vehicle by one

  //See vehicle library for more information
  void details(void);

private:
  unsigned int numOfAirbags_;

};

#endif // CAR_H
