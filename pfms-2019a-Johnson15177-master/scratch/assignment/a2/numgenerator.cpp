#include "numgenerator.h"

NumGenerator::NumGenerator(int seed, int n):
    generator_(seed), distribution_(4.0, 5.0), count_to_append_(n)
{
}

double NumGenerator::RandomNumbers(double mean, double stdDev, double max)
{
    double value;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
    value = distribution_(generator_);
    if (value <0.2)
    {
        value = 0.2;
    }
    if (value > max)
    {
        value = max;
    }
    return value;
}
