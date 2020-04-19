#ifndef RADAR_H
#define RADAR_H

#include "ranger.h"
#include <iostream>

class Radar: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
  Radar();
  //deconstructor
  ~Radar();

  /**
  This function is a virtual function inherite from ranger interface and set anular resolution
  @param[in]    angular resolution
  */
  bool setAngularResolution(unsigned int &angularRes) ;
  /**
  This function is a virtual function inherite from ranger interface and set offset angle
  @param[in]    offset angle
  */
  bool setOffset(int &offSet) ;
  /**
  This function is a virtual function inherite from ranger interface and set field of view
  @param[in]    field of view
  */
  bool setFieldOfView(unsigned int &fov) ;

  /**
  This function put data into the vector container
  */
  void putDataVec();
  //getter



};

#endif // RADAR_H
