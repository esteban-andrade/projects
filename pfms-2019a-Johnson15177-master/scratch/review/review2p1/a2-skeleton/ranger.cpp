#include "ranger.h"
using namespace std;

Ranger::Ranger()    // initialse te ranger with defalut value
{
}

Ranger::~Ranger()
{

}


int Ranger::getNumOfSamples()
{
   return numberOfSamples_ = fov_/angularResolution_;
}

unsigned int Ranger::getFieldOfView(void)
{
    return fov_;
}

string Ranger::getModel()
{
    return model_;
}



double Ranger::getMaxRange()
{
    return maxDistance_;
}

double Ranger::getMinRange()
{
    return minDistance_;
}

double Ranger::getData()
{
    // Setup random number generation using seed from system clock
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // random number set to range from 4 and 5
    std::normal_distribution<double> distribution(4.0, 5.0);

    double data = distribution(generator);

    if(data > maxDistance_){
        data = maxDistance_;
    }
    if(data < minDistance_){
        data =  minDistance_;
    }

    //return value set to random number
    return data;
}

int Ranger::getOffset(void)
{
    return offSet_;
}

unsigned int Ranger::getAngularResolution(void){

    return angularResolution_;
}

vector<double> Ranger::generateData(){

    return value;
}

void Ranger::clear_value()
{
    value.clear();
}





