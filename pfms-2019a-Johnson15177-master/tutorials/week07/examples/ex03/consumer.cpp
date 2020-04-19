#include "consumer.h"
#include <iostream>

Consumer::Consumer()
{

}


// This function consumes the samples
void Consumer::processSamples(Sample &sample) {

    while (true) {
        double value = sample.getSample();
        // This is purely for visualisation IO does not necessarily belong here
        std::cout <<  "sample is:" << value << std::endl;
    }
}
