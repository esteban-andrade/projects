#include <iostream>
#include "sensor.h"

Sensor::Sensor(int seed, std::vector< std::vector< std::vector <double> > > &hyperband): 
    generator_(seed), distribution_(0.0, 255.0)
{
    hyperband_= hyperband;
}

//Function that initialises the 1st band with random values
void Sensor::appendRandomValues(){
    for (int i = 0; i < ROW_SIZE_; i++){
        hyperband_.push_back(rowV_);
        for(int j = 0; j < COLUMN_SIZE_; j++){
            hyperband_[i].push_back(columnV_);
            for(int k = 0; k < FIRST_BAND_; k++){
                hyperband_[i][j].push_back(distribution_(generator_));
            }
        }
    }

//Displays the values of the 1st band Vector
    for (size_t i = 0; i < hyperband_.size(); i++)
        for (size_t j = 0; j < hyperband_[i].size(); j++)
            for (size_t k = 0; k < hyperband_[i][j].size(); k++)
                std::cout << "Hypercube[" << i << "][" << j << "][" << k << "] = " << hyperband_[i][j][k] << std::endl;    
}

//Displays the subsequent bands above, H(i, j, k) = I(i, j, k-1) * 0.8
void Sensor::subHyperband(){
    for(int i = 0; i < ROW_SIZE_; i++){
        for(int j = 0; j < COLUMN_SIZE_; j++){
            for(int k = 1; k < BAND_SIZE_; k++){
                hyperband_[i][j].resize(BAND_SIZE_);
                hyperband_[i][j][k] = hyperband_[i][j][k - 1] * 0.8;
                std::cout << "Hypercube[" << i << "][" << j << "][" << k << "] = " << hyperband_[i][j][k] << std::endl;  
            }
        }
    }
}