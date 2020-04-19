#ifndef SENSOR_H
#define SENSOR_H 

#include <vector>
#include <random>

class Sensor{
    //Initialise the variables for this sensor class, including the variables to create a random number generator 
    private:
        std::default_random_engine generator_;
        std::uniform_real_distribution<double> distribution_; 
        std::vector< std::vector< std::vector <double> > > hyperband_;
        std::vector <std::vector <double> >rowV_;
        std::vector<double> columnV_;
        const int FIRST_BAND_ = 1;
        const int ROW_SIZE_ = 2;
        const int COLUMN_SIZE_ = 4;
        const int BAND_SIZE_ = 8;

    public:
        Sensor(int seed, std::vector< std::vector< std::vector <double> > > &hyperband);
        void appendRandomValues();
        void subHyperband();
};  

#endif 