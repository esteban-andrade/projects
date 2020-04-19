// We need to include the declaration of our new rectangle class in order to use it.
#include "randomvectorfiller.h"
#include <vector>
#include <iostream>
#include <chrono>

void printNumberVector(std::vector numbers){
    for (auto x :number)
}


int main () {
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    int count_to_append = 10;
    RandomVectorFiller filler(seed, count_to_append);

    std::vector<double> numbers;

    filler.appendRandomNumbersTo(numbers);



    return 0;
}
