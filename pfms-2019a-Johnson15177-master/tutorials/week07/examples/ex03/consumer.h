#ifndef Consumer_H
#define Consumer_H
#include "sample.h"
#include <algorithm> // algorithms for sorting

class Consumer
{
public:
  Consumer();

  void processSamples(Sample &sample);

};

#endif // Consumer_H
