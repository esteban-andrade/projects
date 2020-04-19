#include "generator.h"
#include <random>

Generator::Generator(int seed):
    generator_(seed)
{

}

double Generator::RandomNumbers(double mean, double stdDev, double max)
  
{
    /* if the random value is either above or below the limit set it will make them equal to either the max or min value of the sensors. */
    double random;
    std::normal_distribution<double> distribution_(mean, stdDev);
    random = distribution_(generator_);
    if (random < 0.2)
    {
        random = 0.2;
    }
    if (random > max)
    {
        random = max;
    }
    return random;
}

