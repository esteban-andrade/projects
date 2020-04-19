#ifndef MOTORBIKE_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define MOTORBIKE_H

#include <string>
#include "vehicle.h"

class Motorbike: public Vehicle {
public:
  Motorbike(std::string, std::string, std::string, int);
   void details(void);
};

#endif // MOTORBIKE_H
