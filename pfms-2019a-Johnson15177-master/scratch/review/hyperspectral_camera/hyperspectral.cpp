#include "hyperspectral.h"


HyperSpec::HyperSpec(int seed, int n):
    generator_(seed),
    distribution_(0,255),
    num_samples_(n)
{
}

void HyperSpec::arrayFill(double array[])
{
    for(int i = 0; i < num_samples_; i++)
    {
        array[i] = distribution_(generator_);
    }
}

int HyperSpec::setSampleRate()
{
    int sample_rate = 0;
    printf("Enter sample rate (1Hz or 2Hz): ");
    for(sample_rate == 0; sample_rate < 1 || sample_rate > 2;)
    {
        scanf("%d", &sample_rate);
        if(sample_rate < 1 || sample_rate > 2)
            printf("Invalid input, try again: ");
    }
    sample_rate = (1*1000)/sample_rate;
    return sample_rate;
}

void HyperSpec::displayHyperband(double array[], int band)
{
    for(int i = 0; i < 8; i++)
    {
        if(i == 4)
            printf("\n");
        printf("%0.0f\t", array[i]);
    }
    printf("Hyperband: %d", band);
    printf("\n\n");
}

void HyperSpec::nextHyperband(double *p_array[])
{
    for(int i = 0; i < 8; i++)
    {
        *p_array[i] = *p_array[i]*0.8;
    }
}



