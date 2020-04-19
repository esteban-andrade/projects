#ifndef LASER_H
#define LASER_H

#include <chrono>
#include <vector>
#include <string>
#include "ranger.h"
#include "generator.h"
#include "rangerfusion.h"

using namespace std;

/*!
 *  \ingroup    Ranger
 *  \brief      Laser derived class of Ranger
 *  \details
 *  This is the derived class of Ranger.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Laser();
  //! generates data depending on the FOV and angular resolution for laser
  vector<double> generateData();
  //! gets the string of the model for laser
  string getModel(void);
  //! gets the field of view for laser
  unsigned int getFieldOfView(void);
  //! gets the angular resolution for laser
  unsigned int getAngularResolution(void);
  //! gets the offset for laser
  int getOffset(void);
  //! sets the field of view for laser
  bool setFieldOfView(unsigned int);
  //! sets the angular resolution for laser
  bool setAngularResolution(unsigned int);
  //! sets the offset for laser
  bool setOffset(int);
  //! gets the min range for laser
  double getMinRange(void);
  //! gets the max range for laser
  double getMaxRange(void);

private:
  string model; //!< used to return the model
  string model_ = "UTM-XXL";
  
  int FOV;  //!< used to return the FOV
  int FOV_ = 180;
  
  double maxDistance;   //!< used to return the max range
  double minDistance;   //!< used to return the min range
  double max_distance_ = 8.0;
  double min_distance_ = 0.2;
  
  int angRes;   //!< used to return the angular resolution
  int ang_res1_ = 10;
  int ang_res2_ = 30;
  
  int offSet;   //!< used to return the offset
  int offSet_ = 0;
  
};

#endif // LASER_H
