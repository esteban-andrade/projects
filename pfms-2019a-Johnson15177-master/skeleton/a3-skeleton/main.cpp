/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Add information here
 *
 *  @author {TODO: Your student name + id}
 *  @date {TODO}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"

//For example purposes only, this thread gets the friendly aircraft's
//(red triangle) pose every 4 seconds. It plots this pose on the
//simulation (blue triangle) which stays on the image for 1 second, as per the
//'testPose()' documentation in the simualtor class.
void exampleThread(const std::shared_ptr<Simulator> & sim) {
  while(true) {
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    sim->testPose(pose);

    std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
    std::cout << "Friendly {x, y, orientation}:"<< std::endl;
    std::cout << "  - x: " << pose.position.x << "m" << std::endl;
    std::cout << "  - y: " << pose.position.y << "m" << std::endl;
    std::cout << "  - orient: " << pose.orientation << " radians" << std::endl << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
  }
}

void controlThread(const std::shared_ptr<Simulator> & sim) {
  while(true){
    //Feed the watchdog control timer
    double lin = sim->getFriendlyLinearVelocity();
    double ang = sim->getFriendlyAngularVelocity();

    sim->controlFriendly(500, 0.02);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int main(void)
{
  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim));
  threads.push_back(std::thread(exampleThread, sim));

  //Join threads and begin!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
