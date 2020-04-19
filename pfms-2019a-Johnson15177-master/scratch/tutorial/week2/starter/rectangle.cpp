#include "randomvectorfiller.h"
#include <vector>
#include <random>

void populateWithRandomNumbers(double num_array[], int& array_size, int num_elements) {

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0,10.0);
    // generate the required amount of random numbers
    for (int i=array_size; i<array_size+num_elements; i++) {
        num_array[i] = value_distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}



void VectorRandomFiller::RandomVectorFiller(int seed, int n)
{
    seed_ = seed;
    count_to_append_= n;


}

void VectorRandomFiller::appendRandomNumbersTo(std::vector<double> &num_vec)
{
    std::default_random_engine_generator;
    std::normal_distribution<double> distribution (5.0, 2.0);

    for (int i = 0; i < count_to_append; i++)
    num.vec.push_back(distribution(generator));
}
