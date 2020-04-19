#include <random>
#include <iostream> 
#include "ranger.h"
#include "laser.h"

const int laserOffset_ = 0; //Orientation offset
const double laserMaxDistance_ = 8.0;
const double laserMinDistance_ = 0.2;
const int laserFOV_ = 180; //Field Of View
const int laserAngleRes_ = 10;
const int laserNumSamples_ = laserFOV_/laserAngleRes_ + 1; 

Laser::Laser():Ranger(laserAngleRes_, laserOffset_, laserFOV_, 
    laserMaxDistance_, laserMinDistance_, laserNumSamples_){
}
