#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class
class RangerInterface
{
public:
  RangerInterface();

  //!Generates raw data for sensor
  virtual std::vector<double> generateData() = 0;

  //!Essential getter for obtaining internal private variables
  virtual unsigned int getAngularResolution(void) = 0;
  //!Essential getter for obtaining internal private variables
  virtual int getOffset(void) = 0;
  //!Essential getter for obtaining internal private variables
  virtual unsigned int getFieldOfView(void) = 0;
  //!Essential getter for obtaining internal private variables
  virtual double getMaxRange(void) = 0;
  //!Essential getter for obtaining internal private variables
  virtual double getMinRange(void) = 0;

  //!Essential setters for setting internal private variables
  virtual bool setAngularResolution(unsigned int) = 0;
  //!Essential setter for setting internal private variables
  virtual bool setOffset(double) = 0;
  //!Essential setters for setting internal private variables
  virtual bool setFieldOfView(unsigned int) = 0;
};

#endif // RANGERFUSIONINTERFACE_H
