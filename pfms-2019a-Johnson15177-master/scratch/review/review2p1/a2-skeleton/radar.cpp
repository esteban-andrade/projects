#include "ranger.h"
#include "radar.h"
#include <iostream>

#define RADAR_DEFAULT_FIELDOFVIEW 60
#define RADAR_MAX_DISTANCE 16.0
#define RADAR_MIN_DISTANCE 0.2
#define RADAR_DEFAULT_ANGRES 20
#define RADAR_DEFAULT_OFFSET 40

Radar::Radar()  //constructor
{
    //Initialise all the values of laser
    model_ = "RAD-001";
    fov_ = RADAR_DEFAULT_FIELDOFVIEW ;
    maxDistance_ = RADAR_MAX_DISTANCE;
    minDistance_ = RADAR_MIN_DISTANCE;
    angularResolution_ = RADAR_DEFAULT_ANGRES;
    offSet_ = 40;

}

Radar::~Radar() //deconstructor
{

}

bool Radar::setAngularResolution(unsigned int &angularRes)
{
    if(angularRes == RADAR_DEFAULT_ANGRES)
    {
        angularResolution_ = angularRes;
        return true;
    }
    else
    {
         angularResolution_ = RADAR_DEFAULT_ANGRES;
        return false;
    }
}

bool Radar::setOffset(int &offSet)
{
    if(offSet == RADAR_DEFAULT_OFFSET || offSet == 70 || offSet == 100)
    {
    offSet_ = offSet;
    return true;
    }

    else
    {
        offSet = RADAR_DEFAULT_OFFSET;
        return false;
    }


}

bool Radar::setFieldOfView(unsigned int &fov)
{
    if(fov == RADAR_DEFAULT_FIELDOFVIEW)
    {
        fov_ = fov;
        return true;
    }
    else
    {
        fov_ = RADAR_DEFAULT_FIELDOFVIEW;
        return false;
    }
}

void Radar::putDataVec()
{
    for(int i = 0; i < numberOfSamples_; i ++){
        value.push_back(getData());
    }
}

