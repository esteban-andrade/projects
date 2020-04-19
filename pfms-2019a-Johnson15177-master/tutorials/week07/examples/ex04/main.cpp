#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include "safevector.h"


void addRandomThread(SafeVector& vec) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(0,100);
    while (true) {
        vec.addNumber(distribution(generator));
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
}

void pruneValuesThread(SafeVector& vec) {
    while (true) {
        vec.pruneValues(20,80);
    }
}

void pruneLengthThread(SafeVector& vec) {
    while (true) {
        vec.pruneLength(20);
    }
}

int main ()
{
    SafeVector safe_vector("threadsafe vector");

    // Create the threads
    std::thread add_random_thread(addRandomThread, std::ref(safe_vector));
    std::thread prune_values_thread(pruneValuesThread, std::ref(safe_vector));
    std::thread prune_length_thread(pruneLengthThread, std::ref(safe_vector));

    // Wait for the threads to finish (they wont)
    add_random_thread.join();
    prune_values_thread.join();
    prune_length_thread.join();

    return 0;
}
