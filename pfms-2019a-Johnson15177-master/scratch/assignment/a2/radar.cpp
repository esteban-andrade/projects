#include "radar.h"

#include <iostream>

Radar::Radar()
{
    angRes = ang_res1_;
    offSet = offSet_;
    FOV = FOV_;
    model = model_;
    baud = baud1_;
    maxDistance = max_distance_;
    minDistance = min_distance_;
}

double Radar::getMinDistance(void)
{
    return minDistance;
}

double Radar::getMaxDistance(void)
{
    return maxDistance;
}
unsigned int Radar::getAngularResolution()
{
    return angRes;
}

int Radar::getOffset(void)
{
    return offSet;
}

unsigned int Radar::getFieldOfView(void)
{
    return FOV;
}

string Radar::getModel()
{
    return model;
}

int Radar::getBaud(void)
{
    return baud;
}

int Radar::setAngularResolution(int input)
{
    if (input == ang_res1_)
    {
        angRes = ang_res1_;
        return 1;
    }
    else return 0;
}

int Radar::setOffset(int)
{
    return 0;
}

int Radar::setFieldOfView(unsigned int)
{
    return 0;
}

int Radar::setBaud(int input)
{
    return 0;
}
