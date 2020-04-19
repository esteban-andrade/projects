#ifndef RANGER_H
#define RANGER_H

#include <string>
#include "rangerinterface.h"
//#include "numgenerator.h"

using namespace std;

class Ranger: public RangerInterface
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ranger();
  //See rangerinterface.h for more information
  
  virtual string getModel(void) = 0;

 // virtual int getNumberSamples (void) = 0;
  virtual int getBaud (void) = 0;
  
 // virtual int setNumberSamples (int) = 0;
  virtual int setBaud (int) = 0;
  
  virtual double getMinDistance (void) = 0;
  virtual double getMaxDistance (void) = 0;
  
};

#endif // RANGER_H
