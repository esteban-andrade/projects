#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

/*!
 *  \ingroup    Ranger
 *  \brief      Ranger interface base class
 *  \details
 *  This is the base class for the sensors.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */
// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class
class RangerInterface
{
public:
  RangerInterface();

  //Generates raw data for sensor
  //! generates data for laser and radar
  virtual std::vector<double> generateData() = 0;

  //Essential getters for obtaining internal private variables
  //! gets the angular resolution for laser and radar
  virtual unsigned int getAngularResolution(void) = 0;
  //! gets the offset for laser and radar
  virtual int getOffset(void) = 0;
  //! gets the field of view for laser and radar
  virtual unsigned int getFieldOfView(void) = 0;
  //! gets the max range for laser and radar
  virtual double getMaxRange(void) = 0;
  //! gets the min range for laser and radar
  virtual double getMinRange(void) = 0;
  
  //Essential setters for setting internal private variables
  //! sets the angular resolution for laser and radar
  virtual bool setAngularResolution(unsigned int) = 0;
  //! sets the offset for laser and radar
  virtual bool setOffset(int) = 0;
  //! sets the field of view for laser and radar
  virtual bool setFieldOfView(unsigned int) = 0;
  
  
};

#endif // RANGERFUSIONINTERFACE_H
