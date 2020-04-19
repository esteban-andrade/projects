#include "laser.h"
#include "ranger.h"


#define FIELDOFVIEW 180
#define MAX_DISTANCE 8.0
#define MIN_DISTANCE 0.2
#define LASER_OFFSET 0
#define RES_OFFSET_MIN 10
#define RES_OFFSET_MAX 30
Laser::Laser() // constructor
{
    //Initialise all the values of laser
    model_ = "UTM-XXL";
    fov_ = FIELDOFVIEW ;
    maxDistance_ = MAX_DISTANCE;
    minDistance_ = MIN_DISTANCE;
    angularResolution_ = RES_OFFSET_MIN;
    offSet_ = LASER_OFFSET;
}

Laser::~Laser()     // deconstructor
{

}

bool Laser::setAngularResolution(unsigned int &angularRes)
{
    if(angularRes == RES_OFFSET_MIN || angularRes == RES_OFFSET_MAX)
    {
        angularResolution_ = angularRes;
        return true;
    }
    else
    {
        angularResolution_ = LASER_OFFSET;
        return false;
    }


}

bool Laser::setOffset(int &offSet)
{
    if(offSet == LASER_OFFSET)
    {

        offSet_ = offSet;

        return true;
    }
    else
    {
        offSet = LASER_OFFSET;
        return false;
    }
}

bool Laser::setFieldOfView(unsigned int &fov)
{
    if(fov == FIELDOFVIEW)
    {
        fov_ = fov;
        return true;
    }
    else
    {
        fov_ = FIELDOFVIEW;
        return false;
    }


}

void Laser::putDataVec()
{
    for(int i = 0; i < numberOfSamples_+1; i ++)
    {
        value.push_back(getData());
    }

}

