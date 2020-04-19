#include "laser.h"
#include <iostream>

Laser::Laser()
{
    angRes = ang_res1_;
    offSet = offSet_;
     
    model = model_;
    FOV = FOV_;
    baud = baud1_;
    maxDistance = max_distance_;
    minDistance = min_distance_; 
}

double Laser::getMinDistance(void)
{
    return minDistance;
}

double Laser::getMaxDistance(void)
{
    return maxDistance;
}


unsigned int Laser::getAngularResolution()
{
    return angRes;
}


int Laser::getOffset(void)
{
    return offSet;
}

string Laser::getModel()
{
    return model;
}

unsigned int Laser::getFieldOfView(void)
{
    return FOV;
}

int Laser::setFieldOfView(unsigned int)
{
    return 0;
}

int Laser::getBaud(void)
{
    return baud;
}

int Laser::setAngularResolution(int input)
{
    if (input == ang_res1_)
    {
        angRes = ang_res1_;
        return 1;
    }
    else if (input == ang_res2_)
    {
        angRes = ang_res2_;
        return 1;
    }
    else return 0;
}


int Laser::setOffset(int)
{
    return 0;
}




int Laser::setBaud(int input)
{
    if (input == baud1_)
    {
        baud = baud1_;
        return 1;
    }
    else if (input == baud2_)
    {
        baud = baud2_;
        return 1;
    }
    else return 0;
}

