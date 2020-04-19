#ifndef NUMGENERATOR_H
#define NUMGENERATOR_H

#include <random>

class NumGenerator
{
public:
    NumGenerator(int seed, int n);
    double RandomNumbers(double mean, double stdDev, double max);
    
private:
    int count_to_append_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
};

#endif // NUMGENERATOR_H
