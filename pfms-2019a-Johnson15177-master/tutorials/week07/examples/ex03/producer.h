#ifndef PRODUCER_H
#define PRODUCER_H

#include "sample.h"
#include <chrono>
#include <random>

class Producer
{
public:
  Producer();
  void generateSamples(Sample &sample);

private:
    std::normal_distribution<double> distribution_;
    std::default_random_engine generator_;

};

#endif // PRODUCER_H
