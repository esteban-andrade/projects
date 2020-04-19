#include "laser.h"
#include <iostream>
#include "generator.h"
#include "rangerfusioninterface.h"
#define mean 4.0
#define stdDev 5.0

Laser::Laser()
{

    model = model_;
    FOV = FOV_;
    maxDistance = max_distance_;
    minDistance = min_distance_;
    angRes = ang_res1_;
    offSet = offSet_;
}

vector<double> Laser::generateData()
{
    /* calls the generator class which accepts the mean, standard deviation and the specific max range for the laser */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    Generator myGen(seed);
    double max = maxDistance;
    int amount;
    /* creates a vector the size of either 19 or 7 depending on the angular resolution used and fills it with random numbers */
    vector<double> scanVec(amount = (FOV/angRes) + 1);

        for (int i=0; i<amount; i++)
        {
            scanVec.at(i) = myGen.RandomNumbers(mean, stdDev, max);
        }
        return scanVec;
}
/* gets model */
string Laser::getModel()
{
    return model;
}
/* gets field of view */
unsigned int Laser::getFieldOfView(void)
{
    return FOV;
}
/* sets the field of view */
bool Laser::setFieldOfView(unsigned int)
{
    return true;
}
/* gets the minimum range or distance */
double Laser::getMinRange(void)
{
    return minDistance;
}
/* gets the maximum range or distance */
double Laser::getMaxRange(void)
{
    return maxDistance;
}
/* gets the angular resolution */
unsigned int Laser::getAngularResolution(void)
{
    return angRes;
}
/* sets the angular resolution to either 10 or 30 */
bool Laser::setAngularResolution(unsigned int input)
{
    if (input == ang_res1_)
    {
        angRes = ang_res1_;
        return true;
    }
    else if (input == ang_res2_)
    {
        angRes = ang_res2_;
        return true;
    }
    else return false;
}
/* gets the offset */
int Laser::getOffset(void)
{
    return offSet;
}
/* sets the offset */
bool Laser::setOffset(int)
{
    return true;
}













