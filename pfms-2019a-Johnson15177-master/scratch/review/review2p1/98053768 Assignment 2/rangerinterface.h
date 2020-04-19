#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class

/*!
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   N/A
 *  \date      2019
 *  \bug       none reported as of 2019-04-23
 *  \brief     RangerInterface base class
 *  \details
 *  This is an abstract class that stipulates the interface that the Ranger class uses.\n
 */

class RangerInterface
{
public:
  RangerInterface();

  //Generates raw data for sensor
  virtual std::vector<double> generateData() = 0;

  //Essential getters for obtaining internal private variables
  virtual unsigned int getAngularResolution(void) = 0;
  virtual int getOffset(void) = 0;
  virtual unsigned int getFieldOfView(void) = 0;
  virtual double getMaxRange(void) = 0;
  virtual double getMinRange(void) = 0;

  //Essential setters for setting internal private variables
  virtual bool setAngularResolution(unsigned int) = 0;
  virtual bool setOffset(int) = 0;
  virtual bool setFieldOfView(unsigned int) = 0;
};

#endif // RANGERFUSIONINTERFACE_H
