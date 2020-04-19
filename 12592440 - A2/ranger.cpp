#include <random>
#include <vector>
#include <iostream>
#include <chrono>
#include <sstream>
#include "ranger.h"
#include "laser.h"

Ranger::Ranger(int angleRes, int offset, double fieldOfView, double maxDist, double minDist, int numSamples):
    rangerAngularRes_(angleRes), rangerOffset_(offset), rangerFOV_(fieldOfView), rangerMaxDist_(maxDist), rangerMinDist_(minDist),rangerNumSamples_(numSamples)
{
}

Ranger::Ranger(){
    std::cout << "There are no inputs to the signatures" << std::endl;
}

bool Ranger::setAngularResolution(unsigned int angle){
    rangerAngularRes_ = angle;
}

bool Ranger::setOffset(double offset){
   rangerOffset_ = offset;    
}

bool Ranger::setFieldOfView(unsigned int fieldOfView){
    rangerFOV_ = fieldOfView;
}

unsigned int Ranger::getAngularResolution(void){
    return rangerAngularRes_;
}

unsigned int Ranger::getFieldOfView(void){
    return rangerFOV_;
}

int Ranger::getOffset(void){
    return rangerOffset_;
}

double Ranger::getMaxRange(){
    return rangerMaxDist_;
}

double Ranger::getMinRange(){
    return rangerMinDist_;
}

std::vector<double> Ranger::generateData(){
    rangerData_.clear();
    rangerData_.reserve(rangerNumSamples_); //Empties the vector container then reserves the size it needs by the number of sample to be obtained

    for(unsigned int it = 0; it < rangerNumSamples_; it++){
        rangerData_.push_back(0);
    }
    
    for(unsigned int i = 0; i < rangerNumSamples_; i++){
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::normal_distribution<double> distribution (4.0, 5.0); 
        double data_ = distribution(generator); //Generates random values from a normal distribution
            if(data_ <= rangerMaxDist_ && data_ >= rangerMinDist_){ //filters the random data if it goes over or below the boundary it will receive a capped value
                rangerData_.at(i) = data_;
            } else if(data_ >= rangerMaxDist_){
                rangerData_.at(i) = rangerMaxDist_;
            } else {
                rangerData_.at(i) = rangerMinDist_;
            }
    }
    return rangerData_;
}