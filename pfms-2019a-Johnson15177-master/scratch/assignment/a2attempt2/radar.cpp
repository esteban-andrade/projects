#include "radar.h"
#include <iostream>
#include "generator.h"
#define mean 4.0
#define stdDev 5.0

Radar::Radar()
{
    model = model_;
    FOV = FOV_;
    maxDistance = max_distance_;
    minDistance = min_distance_;
    angRes = ang_res1_;
    offSet = offSet_;

}

vector<double> Radar::generateData()
{
    /* calls the generator class which accepts the mean, standard deviation and the specific max range for the radar */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    Generator myGen(seed);
    double max = maxDistance;
    int amount;
    /* creates a vector the size of either 3 depending on the angular resolution used and fills it with random numbers */
    vector<double> scanVec(amount = (FOV/angRes));

    for (int i=0; i<amount; i++)
    {
        scanVec.at(i) = myGen.RandomNumbers(mean, stdDev, max);
    }
    return scanVec;
}
/* gets the model */
string Radar::getModel()
{
    return model;
}
/* gets field of view */
unsigned int Radar::getFieldOfView(void)
{
    return FOV;
}
/* sets field of view */
bool Radar::setFieldOfView(unsigned int)
{
    return 1;
}
/* gets minium range or distance */
double Radar::getMinRange(void)
{
    return minDistance;
}
/* gets maximum range or distance */
double Radar::getMaxRange(void)
{
    return maxDistance;
}
/* gets the angular resolution */
unsigned int Radar::getAngularResolution(void)
{
    return angRes;
}
/* sets the angular resolution */
bool Radar::setAngularResolution(unsigned int input)
{
    return true;
}
/* gets offset */
int Radar::getOffset(void)
{
    return offSet;
}
/* sets the offset between -120 to 120 and only accepts inputs that are a multiple of 10 */
bool Radar::setOffset(int input)
{
    if (-120<=input && input<=120 && input % 10 == 0)
    {
        offSet_ = input;
        offSet = offSet_;
        return true;
    }
    else return false;
}
