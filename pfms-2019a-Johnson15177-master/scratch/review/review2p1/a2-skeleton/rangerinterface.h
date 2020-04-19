#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

/*!
 *  \ingroup   ranger RangerInterface
 *  \brief     RangerInterface base class
 *  \details
 *  This class is the base class for all sensors \n
 *  \author    Nuo
 *  \version   1.02-1
 *  \date      2019
 *  \pre       none.
 *  \warning
 */
// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class
class RangerInterface
{

public:
  RangerInterface();
  /**
  This function is Essential getters for obtaining internal private variables
  */
  virtual std::vector<double> generateData() = 0;

 //Essential getters for obtaining internal private variables
  /**
  This function is Essential getters for obtaining internal private variables
  */
  virtual unsigned int getAngularResolution(void) = 0;
  /**
  This function is Essential getters for obtaining internal private variables
  */
  virtual int getOffset(void) = 0;
  /**
  This function is Essential getters for obtaining internal private variables
  */
  virtual unsigned int getFieldOfView(void) = 0;
  /**
  This function is Essential getters for obtaining internal private variables
  */
  virtual double getMaxRange(void) = 0;
  /**
  This function is Essential getters for obtaining internal private variables
  */
  virtual double getMinRange(void) = 0;

 //Essential setters for setting internal private variables
  /**
  This function is Essential setters for setting internal private variables
  */
  virtual bool setAngularResolution(unsigned int &angularResolution) = 0;
  /**
  This function is Essential setters for setting internal private variables
  */
  virtual bool setOffset(int &offSet) = 0;
  /**
  This function is Essential setters for setting internal private variables
  */
  virtual bool setFieldOfView(unsigned int &fov) = 0;
};

#endif // RANGERFUSIONINTERFACE_H
