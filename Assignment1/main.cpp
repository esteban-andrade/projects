#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "sensor.h"

void userInput(std::string &sampleRate){
    std::cout << "Hello, Please input your sampling rate of either 1Hz or 2Hz:" << std::endl;
    do
    {
        std::cin >> sampleRate;
        std::cout << "Entered: " << sampleRate << std::endl;
        if (sampleRate == "1Hz" || sampleRate == "2Hz")
        {
            return;
        }
        else
        {
            std::cout << "Input proper sample rate of either 1Hz or 2Hz" << std::endl;
        }
    } while (sampleRate != "1Hz" || sampleRate != "2Hz");

}

int main()
{
    std::string sampleRate;
    userInput(sampleRate); //Asks user to input either 1Hz or 2Hz

    do
    {   
        //Decides from sample rate how often the values will be shown
        if (sampleRate == "1Hz")
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        //creates new seed to obtain new random values
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        
        //creates a 3D vector
        std::vector<std::vector<std::vector<double>>> hypercube;

        //creates an object called sensor
        Sensor sensor(seed, hypercube);

        //Gives random values to the first band
        std::cout << "1st Band Random Values" << std::endl;
        sensor.appendRandomValues(); 

        // multiplies 0.8 to the subsequent bands below, the ouput will show: Hypercube [row][column][band] = value
        std::cout << "Subsequent Bands Above" << std::endl;
        sensor.subHyperband(); 
    } while (1);
}