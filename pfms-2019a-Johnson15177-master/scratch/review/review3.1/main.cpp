/*! @file
 *
 *  @brief PFMS Assignment 3.
 *
 *
 *
 *  @author {Samir Djulamerovic 12544920}
 *  @date {19/05/19}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "autopilot.h"

std::shared_ptr<Simulator> sim(new Simulator());
Autopilot bogie(sim);

/* This thread prints bogie data and shows the target point on the map*/
void bogieInfoThread(const std::shared_ptr<Simulator> & sim) {
  while(true)
  {
    sim->testPose(bogie.getBogiePose());

    std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
    std::cout << "Estimated Bogie {x, y, orientation}:"<< std::endl;
    std::cout << "  - x: " << bogie.getBogiePose().position.x << "m" << std::endl;
    std::cout << "  - y: " << bogie.getBogiePose().position.y << "m" << std::endl;
    std::cout << "  - orient: " << bogie.getBogiePose().orientation << " radians" << std::endl << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

/* This thread handles the generation of estimated bogie positions*/
void setBogieThread()
{
    while(true)
    {
        bogie.setBogiePose();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/* This thread determines the linear and angular velocity to take in order to reach the bogie*/
void pursueThread(const std::shared_ptr<Simulator> & sim)
{
    while(true)
    {
        bogie.setPursue(bogie.getBogiePose().position.x, bogie.getBogiePose().position.y);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/* This thread feeds the watchdog timer with the velocities obtained from the pursueThread*/
void controlThread(const std::shared_ptr<Simulator> & sim) {
  while(true)
  {
    sim->controlFriendly(bogie.getLinControl(), bogie.getAngControl());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int main(void)
{
  std::vector<std::thread> threads;
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim));
  threads.push_back(std::thread(bogieInfoThread, sim));
  threads.push_back(std::thread(pursueThread, sim));
  threads.push_back(std::thread(setBogieThread));

  for(auto & t: threads)
  {
    t.join();
  }

  return 0;
}
