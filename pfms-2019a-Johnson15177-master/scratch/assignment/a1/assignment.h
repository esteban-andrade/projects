#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <thread>

class HyperCamera
{
public:
    HyperCamera(int seed, int n);           
    void RandNumTo(std::vector<double> &num_vec1);
private:
    int Max_Amount_;
    std::default_random_engine generator_;
    //using uniform real distribution for the random number generator
    std::uniform_real_distribution<double> distribution_;   
};

#endif // ASSIGNMENT_H
