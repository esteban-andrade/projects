#include <iostream>
#include <thread>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting


#include "sample.h"

using namespace std;

// The function generates samples
void generateSamples(Sample &sample) {

    // Setup and seed our random number generator
    std::default_random_engine generator(
            std::chrono::system_clock::now().time_since_epoch().count());
    // Set mean and std dev of our distribution
    std::normal_distribution<double> distribution(6.0, 5.0);

    while (true) {
      // This delay is included to emulate the data rate of a sensor
        std::this_thread::sleep_for (std::chrono::milliseconds(500));

        cout << "Generating sample" << endl;
        double value= distribution(generator);

        sample.addSample(value);
    }
}

// This function consumes the samples
void processSamples(Sample &sample) {
    while (true) {
        double value = sample.getSample();
        cout <<  "sample is:" << value << endl;
    }
}


int main ()
{
    Sample sample;

    // Create the threads
    thread inc_thread(generateSamples ,ref(sample));
    thread print_thread(processSamples,ref(sample));

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



