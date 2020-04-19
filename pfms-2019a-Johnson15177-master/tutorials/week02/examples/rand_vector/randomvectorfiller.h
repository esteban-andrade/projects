#ifndef RANDOMVECTORFILLER_H
#define RANDOMVECTORFILLER_H

#include <vector>
#include <random>


class RandomVectorFiller
{
public:
    RandomVectorFiller(int seed, int n);
    void appendRandomNumbersTo(std::vector<double> &num_vec);

private:
    int count_to_append_;

    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
};

#endif // RANDOMVECTORFILLER_H
