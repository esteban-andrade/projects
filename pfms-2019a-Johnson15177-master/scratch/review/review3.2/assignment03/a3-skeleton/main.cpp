/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  The aim of this assignment was to develop an understanding of utilising classes, abstraction, data structures, threading and data synchronisation.
 *
 *  In this scenario an aircraft is patrolling airspace (red arrow) surrounding a base station (black dot) with the intent of localising any enemy aircraft (green arrow) that enters the airspace and shadowing it.
 *
 *  The intent of this main.cpp document is to establish the required threads needed to localise the enemy aircraft and control the friendly aircraft within the specified parameters.
 *
 *  @author {Liam Thurston (98102534)}
 *  @date {19/05/2019}
*/
#include <thread>
#include <vector>
#include <iostream>
#include <cmath>
#include "simulator.h"
#include "rangebuffer.h"
#include "controller.h"


//!
//! \brief friendlyPositionThread
//!
//! This thread is run in parallel to print to the user the current time and position of the friendly aircraft. Whilst not critical to the operation of the aircraft, this thread provides useful feedback to the user during operation
//!
//! \param sim - Simulator shared pointer used to access member functions of the simulator
//!
void friendlyPositionThread(const std::shared_ptr<Simulator> & sim) {
  while(true) {
    //Get the friendly aircraft's position and orientation
    Pose pose_friendly = sim->getFriendlyPose();

    // Print friendly aircraft position information to the terminal
    std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
    std::cout << "Friendly {x, y, orientation}:"<< std::endl;
    std::cout << "  - x: " << pose_friendly.position.x << "m" << std::endl;
    std::cout << "  - y: " << pose_friendly.position.y << "m" << std::endl;
    std::cout << "  - orient: " << pose_friendly.orientation << " radians" << std::endl << std::endl;

    // Sleep so that the terminal is not flooded with text.
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
}


//!
//! \brief controlThread
//!
//! This thread is created to provide control to the friendly aircraft. This thread solely sends control signals to the friendly aircraft from within the Controller class.
//!
//! \param friendly_controller - Controller object to be used to control the friendly aircraft
//!
void controlThread(Controller & friendly_controller) {
  while(true){
    //Feed the watchdog control timer

    friendly_controller.controlFriendly();

    // Sleep to emulate the required rate of control signals (50 ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}


//!
//! \brief baseRangeThread
//!
//! The idea of this thread is to simulate the radar on the base station.
//!
//! \param bogey_range_data - RangeBuffer object used to store the radar data from each of the sensors.
//!
void baseRangeThread(RangeBuffer & bogey_range_data){
    while(true){
        // Call function to generate base radar data
        bogey_range_data.addBaseData();
    }
}


//!
//! \brief friendlyRangeThread
//!
//! The idea of this thread is to simulate the radar on the friendly aircraft.
//!
//! \param bogey_range_data - RangeBuffer object used to store the radar data from each of the sensors.
//!
void friendlyRangeThread(RangeBuffer &bogey_range_data){
    while(true){
        // Call function to generate friendly aircraft radar data
        bogey_range_data.addFriendlyData();
    }
}


//!
//! \brief calculateThread
//!
//! The idea of this thread is to calculate the possible bogey positions and the control actions necessary to chase the bogey aircraft
//!
//! \param friendly_controller - Controller object to be used to control the friendly aircraft
//! \param bogey_range_data - RangeBuffer object which stores the radar data from each sensor
//!
void calculateThread(Controller &friendly_controller, RangeBuffer & bogey_range_data){
    while(true){
        // Call function to calculate bogey position. Pass RangeBuffer object so the function is able to call for the radar data
        friendly_controller.calculateBogeyPosition(std::ref(bogey_range_data));

        // Call function to calculate control actions
        friendly_controller.calculateControlActions();
    }
}


//!
//! \brief main
//!
//! Main loop of the program used to set up the threads and initialise passing of the appropriate data between the classes.
//!
//! \return 0 - int value used to terminate the main function. In this instance, the main will only terminate when the user inputs ctrl+C
//!
int main(void)
{
  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());

  // Create RangeBuffer object and pass the simulator to the constructor
  RangeBuffer bogey_range_data(sim);

  // Create controller object to control the friendly aircraft and pass the simulator to the constructor
  Controller friendly_controller(sim);

  // Generate all the threads required for parallel execution and store them in a vector of threads.
  threads.push_back(std::thread(controlThread, std::ref(friendly_controller)));
  threads.push_back(std::thread(baseRangeThread, std::ref(bogey_range_data)));
  threads.push_back(std::thread(friendlyRangeThread, std::ref(bogey_range_data)));
  threads.push_back(std::thread(calculateThread, std::ref(friendly_controller), std::ref(bogey_range_data)));
  threads.push_back(std::thread(friendlyPositionThread, sim));
  threads.push_back(sim->spawn());

  //Join threads and begin!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
