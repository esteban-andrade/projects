#include <iostream>
#include <chrono>
#include <thread>
#include "hyperspectral.h"

using namespace std;

int main()
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    HyperSpec array_gen(seed, 8);
    double array[8];
    int end = 0;
    int sample_sequence_number = 0;
    int sample_rate = 0;
    double *p_array[8];

    //Enter sample rate and calculate rate in milliseconds
    sample_rate = array_gen.setSampleRate();

    //Loop for program indefinitely
    while(end == 0)
    {
        //Filling array with random numbers
        array_gen.arrayFill(array);
        int band = 1;

        //Outputting the sequence number of the sample
        sample_sequence_number++;
        printf("\nSample Sequence: %d\n\n", sample_sequence_number);

        //Displaying the output for randomised hypercubes
        while(band <= 8)
        {
            //Displaying first hyperband
            array_gen.displayHyperband(array, band);

            //Calculating the next hyperband
            for(int i = 0; i < 8; i++)
            {
                p_array[i] = &array[i];
            }
            array_gen.nextHyperband(p_array);
            band++;
        }
        //Delay based on the sample rate entered by user
        printf("\n\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_rate));
    }
    return 0;
}
