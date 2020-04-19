#include "producer.h"
#include <iostream>
#include <thread>

Producer::Producer() :
    generator_(std::chrono::system_clock::now().time_since_epoch().count()),
    distribution_(6.0, 5.0)
{

}

// The function generates samples
void Producer::generateSamples(Sample &sample) {

    while (true) {
      // This delay is included to emulate the data rate of a sensor
        std::this_thread::sleep_for (std::chrono::milliseconds(500));

        // This is purely for visualisation IO does not necessarily belong here
        std::cout << "Generating sample" << std::endl;
        double value= distribution_(generator_);

        sample.addSample(value);
    }
}
