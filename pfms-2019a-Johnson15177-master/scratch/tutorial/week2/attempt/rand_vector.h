#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>

class RandVector
{
private:
    int count_to_append_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;

public:
    RandVector(int seed, int n);
    void appendRandomNumbersTo(std::vector<double> &num_vec);
};
