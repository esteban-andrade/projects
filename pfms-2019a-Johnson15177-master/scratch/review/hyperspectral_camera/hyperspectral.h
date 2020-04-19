#ifndef HYPERSPECTRAL_H
#define HYPERSPECTRAL_H
#include <random>

class HyperSpec
{
public:
    HyperSpec(int seed, int n);
    HyperSpec();
    void arrayFill(double array[]);
    int setSampleRate();
    void displayHyperband(double array[], int band);
    void nextHyperband(double *p_array[]);

private:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> distribution_;
    int num_samples_;
};

#endif // HYPERSPECTRAL_H

