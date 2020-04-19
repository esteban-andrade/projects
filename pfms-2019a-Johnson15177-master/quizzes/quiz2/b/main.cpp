// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include<vector>

int main (void) {

    //Some specifications provided here, though you can use any of your own

    //Mercedes C180 Compressor.
    // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    //Bugatti Veyron Super Sport.
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg


    //!TODO
    // Create a Bugatti and Mercedes object
    Car bugatti("Bugatti", "Veyron Super Sport", 1.19, 2.0, 1200, 0.35, 2200);
    Car mercedes("Mercedes", "C180 Compressor", 1.45, 1.77, 143, 0.29, 1200); 

    bool still_racing = true;

    //Slow down the thread for 100 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(200));


    //! Race until races ends
    while (still_racing){
        //!TODO 
        //! Accelerate cars to top speed
        bugatti.accelerate();
        mercedes.accelerate();
        //! Decelearate after reachig top speed to zero
        bugatti.decelerate();
        mercedes.decelerate();
        //! Print when each car reach top speed
        std::cout << "Bugatti reaches top speed at" << std::endl;
        std::cout << "Mercedes reaches top speed at" << std::endl;
        //! Print the car details of fisrt car to reach speed of zero
        //! Print the current speed of all other cars 

      //Slow down the thread for 100 miscroseconds
      std::this_thread::sleep_for(std::chrono::microseconds(50));


      }
    return 0;
}
