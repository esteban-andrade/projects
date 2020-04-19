#ifndef GENERATOR_H
#define GENERATOR_H

#include <vector>
#include <random>

/*!
 *  \ingroup    random Generator
 *  \brief      Random Number Generator
 *  \details
 *  This is the random number generator class.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */


/* the generator class is used to generate normal distributed random numbers */
class Generator
{
public:
   //! Takes a seed for random values.
    Generator(int seed);
   //! takes the mean, standard deviation and max value
   //! returns the random number
    double RandomNumbers(double mean, double stdDev, double max);
   
private:
    int numbers_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;//!< sets the random generator to take normal distribution
    
    double minDistance;
    double maxDistance;
};

#endif // GENERATOR_H
