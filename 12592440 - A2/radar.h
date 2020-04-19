#ifndef RADAR_H
#define RADAR_H

#include "ranger.h"
#include <string>

class Radar: public Ranger
{
   public:
  //! Initialises the Radar default object and sends it to the ranger class
  /*!
    \sa Ranger()
  */
    Radar();

};

#endif // RADAR_H
