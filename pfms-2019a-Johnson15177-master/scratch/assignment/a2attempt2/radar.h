#ifndef RADAR_H
#define RADAR_H

#include<chrono>
#include <vector>
#include <string>
#include "ranger.h"
#include "generator.h"
#include "rangerfusion.h"

using namespace std;

/*!
 *  \ingroup    Ranger
 *  \brief      Radar derived class of Ranger
 *  \details
 *  This is the derived class of Ranger.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */

class Radar: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
  Radar();
  //! generates data depending on the FOV and angular resolution for radar
  vector<double> generateData();
  //! gets the string of the model for radar
  string getModel(void);
  //! gets the field of view for radar
  unsigned int getFieldOfView(void);
  //! gets the angular resolution for radar
  unsigned int getAngularResolution(void);
  //! gets the offset for radar
  int getOffset(void);
  //! sets the field of view for radar
  bool setFieldOfView(unsigned int);
  //! sets the angular resolution for radar
  bool setAngularResolution(unsigned int);
  //! sets the offset for radar
  bool setOffset(int);
  //! gets the min range for radar
  double getMinRange(void);
  //! gets the max range for radar
  double getMaxRange(void);

private:
  string model; //!< used to return the model
  string model_ = "RAD-001";
  
  int FOV;  //!< used to return the FOV
  int FOV_ = 60;
  
  double maxDistance;   //!< used to return the max range
  double minDistance;   //!< used to return the min range
  double max_distance_ = 16.0;
  double min_distance_ = 0.2;
  
  int angRes;   //!< used to return the angular resolution
  int ang_res1_ = 20;
  
  int offSet;   //!< used to return the offset
  int offSet_ = 0;

  
  
  
  
};

#endif // RADAR_H
