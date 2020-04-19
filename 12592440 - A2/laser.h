#ifndef LASER_H
#define LASER_H

#include "ranger.h"
#include <string>
#include <vector>

class Laser: public Ranger
{
  public:
  //! Initialises the Laser default object and sends it to the ranger class
  /*!
    \sa Ranger()
  */
    Laser(); 
  
};

#endif // LASER_H
