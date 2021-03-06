#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class
class RangerInterface
{
public:
  RangerInterface();

  //Generates raw data for sensor
 // virtual std::vector<double> generateData() = 0;

  //Essential getters for obtaining internal private variables
  virtual unsigned int getAngularResolution(void) = 0;
  virtual int getOffset(void) = 0;
  virtual unsigned int getFieldOfView(void) = 0;

  //Essential setters for setting internal private variables
  virtual int setAngularResolution(int) = 0;
  virtual int setOffset(int) = 0;
  virtual int setFieldOfView(unsigned int) = 0;
  
  
};

#endif // RANGERFUSIONINTERFACE_H
