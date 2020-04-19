#ifndef MOTORBIKE_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define MOTORBIKE_H

#include <string>
#include "vehicle.h"

class Motorbike: public Vehicle {
public:
  Motorbike(std::string, std::string, std::string, int);
  
  void accelerate(void); //Increment the currentSpeed of the vehicle by one
  void decelerate(void); //Decrement the currentSpeed of the vehicle by one

  
  void details(void);
  
  
private:
  
  
  
};

#endif // MOTORBIKE_H
