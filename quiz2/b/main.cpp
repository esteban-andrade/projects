// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread> // std::this_thread::sleep_for
#include <vector>

int main(void)
{

  //Some specifications provided here, though you can use any of your own

  //Mercedes C180 Compressor.
  // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

  //Bugatti Veyron Super Sport.
  // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg

  //!TODO
  // Create a Bugatti and Mercedes object
  Car Bugatti("Veyron", "Super Sport", 1.19, 2.00, 1200, 0.35, 2200);
  Car Mercedes("C180", "Compressor", 1.45, 1.77, 143, 0.29, 1200);

  std::vector<Car> cars;

  cars.push_back(Mercedes);
  cars.push_back(Bugatti);

  //OR

  // cars.push_back(Car("bugatti", "Veyron", 1.19, 2.00, 1200, 0.35, 2200));
  // cars.push_back(Car("Mercedes", "c180", 1.45, 1.77, 143, 0.29, 1200));

  std::vector<double> reached_top_speed;
  for (unsigned int i = 0; i < cars.size(); i++)
  {
    reached_top_speed.push_back(cars[i].calculateTopSpeed());
  }
  std::cout << reached_top_speed[0];
  std::cout << reached_top_speed[1];

  bool still_racing = true;

  //Slow down the thread for 100 miscroseconds
  std::this_thread::sleep_for(std::chrono::microseconds(100));

  //! Race until races ends

  while (still_racing)
  {
    for (unsigned int i = 0; i < cars.size(); i++)
    {
      std::cout << "Car #" << i << "= " << cars[i].getCurrentSpeed() << std::endl;
      if (cars[i].getDirection())
      {
        cars[i].accelerate();
        if (cars[i].getCurrentSpeed() >= reached_top_speed[i])
        {
          std::cout << cars[i].getMake() << " has reached top speed" << std::endl;
          cars[i].getDirection(0);
          //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
      }
      else if(!(cars[i].getDirection()))
      {
        cars[i].decelerate();
        if (cars[i].getCurrentSpeed() <= 0.0)
        {
          std::cout << cars[i].getMake() << " has reached speed zero " << std::endl;
          cars[1].getCurrentSpeed();
          cars[0].getCurrentSpeed();
          return 0;
        }
        
      }

      /*
      cars[i].accelerate();
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      if (reached_top_speed.at(i) == cars[i].calculateTopSpeed())
      {
        std::cout << cars[i].getMake() << " has reached top speed" << std::endl;
        cars.at(i).decelerate();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
        if (cars.at(i).getCurrentSpeed() < 0.00)
        {
            std::cout << cars[i].getMake() << " has reached speed zero" << std::endl;
            std::cout << cars[i].getCurrentSpeed();
             std::this_thread::sleep_for(std::chrono::microseconds(100));
             still_racing = false;
        }
      }*/
      //!TODO
      //! Accelerate cars to top speed
      //! Decelearate after reachig top speed to zero
      //! Print when each car reach top speed
      //! Print the car details of fisrt car to reach speed of zero
      //! Print the current speed of all other cars

      //Slow down the thread for 100 miscroseconds
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
  return 0;
}
