#include <chrono>
#include <random>
#include <string>
#include <vector>
#include "rangerinterface.h"
#include "ranger.h"

// Default constructor to set all sensor attributes to a default value
Ranger::Ranger() :
    model_("Generic ranger"),
    field_of_view_(180),
    angular_res_(30),
    offset_(0),
    min_range_(0.2),
    max_range_(8.0),
    current_angle_(0),
    sample_num_(1),
    total_readings_(field_of_view_/angular_res_ + 1),
    mean_(4.0),
    std_dev_(5.0),
    generator_(seed_),
    value_distribution_(mean_,std_dev_)
{ }

//----------------------------------GETTERS----------------------------------//
std::string Ranger::getModel(void) {
    return model_;
}

unsigned int Ranger::getAngularResolution(void) {
    return angular_res_;
}

int Ranger::getOffset(void) {
    return offset_;
}

unsigned int Ranger::getFieldOfView(void) {
    return field_of_view_;
}

double Ranger::getMaxRange(void) {
    return max_range_;
}

double Ranger::getMinRange(void) {
    return min_range_;
}

double Ranger::getMean(void) {
    return mean_;
}

double Ranger::getStdDev(void) {
    return std_dev_;
}

int Ranger::getSampleNum(void) {
    return sample_num_;
}

//----------------------------------SETTERS----------------------------------//
bool Ranger::setAngularResolution(unsigned int ang_res) {
    angular_res_ = ang_res;
    return true;
}

bool Ranger::setOffset(int offset) {
    offset_ = offset;
    return true;
}

bool Ranger::setFieldOfView(unsigned int fov) {
    field_of_view_ = fov;
    return true;
}

bool Ranger::setMean(double mean) {
    if (mean > 0) {
        // set new mean if value is greater than zero
        mean_ = mean;
        return true;
    }
    else {
        // set default mean otherwise
        mean_ = 4.0;
        return false;
    }
}

bool Ranger::setStdDev(double sd) {
    if (sd > 0) {
        // set new mean if value is greater than zero
        std_dev_ = sd;
        return true;
    }
    else {
        // set default mean otherwise
        std_dev_ = 5.0;
        return false;
    }
}

bool Ranger::setSampleNum(int sample_num) {
    if (sample_num > 0) {
        sample_num_ = sample_num;
        return true;
    }
    else {
        sample_num_ = 1;
        return false;
    }
}


//------------------------------DATA GENERATOR-------------------------------//
std::vector<double> Ranger::generateData() {
    std::vector<double> data;

    // Make vector large enough to hold all the readings required
    data.reserve(total_readings_);
    // Generate data
    for(int i = 0 ; i < data.capacity() ; i++) {
        data.push_back(Ranger::generateRandNum());
    }
    sample_num_++;
    return data;
}


//-----------------------------PROTECTED METHODS-----------------------------//
double Ranger::generateRandNum(void) {
    // Generate value
    double rand = value_distribution_(generator_);

    // Only return value if it is within the range
    if (rand >= min_range_ && rand <= max_range_) return rand;
    // return minimum value if it is below than the range
    else if (rand <= min_range_) return min_range_;
    // return maximum value if it is above than the range
    else if (rand >= max_range_) return max_range_;
}
