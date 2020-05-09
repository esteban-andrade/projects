#include <thread>
#include <vector>
#include <iostream>
#include <deque>
#include <condition_variable>
#include <queue>
#include "dataSynch.h"
#include "simulator.h"
#include "seek.h"
#include "navigate.h"

#define BUFFER 10


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

void controlThread(const std::shared_ptr<Simulator> & sim, std::condition_variable& cv2, std::mutex& mtx,AircraftContainer& friendly,std::vector<long>& baseCompare, std::vector<long>& friendCompare, DataSynch& data) {
    Seek seek;
    Navigate navigate;
    Pose bogie0;
    Pose bogie1;
    bool start = true;
  while(true){
    //Feed the watchdog control timer<
    friendly.a.pose = sim->getFriendlyPose();
    std::vector<long> base =data.getBaseCompare();
    std::vector<long> friendData = data.getFriendCompare();
    std::deque<Pose> correctBogie = seek.getCorrectBogie();
    navigate.checkAirspace(sim, friendly.a.pose);
    while(!base.empty() && !friendData.empty()){
        std::cout << "Synched Data:   " << "Base: " << base[0] << "= " << base[1] << "    Friend: " << friendData[0] << "= " << friendData[1] << std::endl;
        seek.circle_circle_intersection(friendly.a.pose.position, sim->BSTATION_LOC, friendData[1], base[1], bogie0, bogie1);
        seek.findPosition(bogie0, bogie1);
        seek.pushBogies(start ,bogie0, bogie1);
        seek.getPosition(bogie0, bogie1);
        correctBogie = seek.getCorrectBogie();
        sim->testPose(correctBogie.front());
        base = data.getBaseCompare();
        friendData = data.getFriendCompare();
        data.resetData();
    } 

    navigate.getVectorOrientation(correctBogie, friendly.a.pose);
    navigate.move();
    double lin = navigate.getLinSpeed();
    double ang = navigate.getAngVel();
    
    sim->controlFriendly(lin, ang);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
  }
          
}

void showData(const std::shared_ptr<Simulator> & sim, std::condition_variable& cv1, std::condition_variable& cv2, std::deque<RangeStamped>& baseContainer,AircraftContainer& friendly, std::vector<double>& timeFriend, std::vector<double>& rangeFriend, std::vector<long>& baseCompare, std::vector<long>& friendCompare, DataSynch& data){
    while(true){
        {
            std::unique_lock<std::mutex> locker(friendly.access);
            data.showData(locker);
        }
    }
}

double interpolate(long xL, long xR, double yL, double yR, double x, bool extrapolate){
    if ( !extrapolate )                                                         
    {
        if ( x < xL ) yR = yL;
        if ( x > xR ) yL = yR;
    }
    double dydx = ( yR - yL ) / ( xR - xL );                                   
    return yL + dydx * ( x - xL );       
}

void rangeBaseToBogie(const std::shared_ptr<Simulator> & sim, std::condition_variable& cv, std::condition_variable& cv1, AircraftContainer& friendly, std::deque<RangeStamped>& baseContainer, DataSynch& data){
    RangeStamped baseToBogie;
    while(true){
        baseToBogie = sim->rangeToBogieFromBase();
        std::unique_lock<std::mutex> locker(friendly.access);
        data.getBaseData(baseToBogie);
    }
}

void rangeFriendlyToBogie(const std::shared_ptr<Simulator>& sim, std::condition_variable& cv, std::condition_variable& cv2,AircraftContainer& friendly, std::deque<RangeStamped>& friendlyContainer, std::vector<double>& timeFriend, std::vector<double>& rangeFriend, DataSynch& data){
    RangeStamped friendlyToBogie;

    while(true){
        friendlyToBogie = sim->rangeToBogieFromFriendly();
        std::unique_lock<std::mutex> locker(friendly.access);
        data.getFriendlyData(friendlyToBogie);
        data.checkFriendSize(locker);
    }
}

int main(void)
{
    DataSynch data;

    std::vector<std::thread> threads;
    AircraftContainer friendly;
    std::mutex mtx;

    
    std::vector<long> baseCompare;
    std::vector<long> friendCompare;
    std::deque<RangeStamped> friendlyContainer;
    std::vector<double> timeFriend;
    std::vector<double> rangeFriend;
    std::deque<RangeStamped> baseContainer(BUFFER);

    std::condition_variable cv;
    std::condition_variable cv1;
    std::condition_variable cv2;

    //Create a shared pointer for the simulator class
    std::shared_ptr<Simulator> sim(new Simulator());
    threads.push_back(sim->spawn());
    threads.push_back(std::thread(controlThread, sim, std::ref(cv1), std::ref(mtx),std::ref(friendly), std::ref(baseCompare), std::ref(friendCompare),std::ref(data)));
    threads.push_back(std::thread(rangeFriendlyToBogie, sim, std::ref(cv), std::ref(cv2),std::ref(friendly), std::ref(friendlyContainer), std::ref(timeFriend), std::ref(rangeFriend), std::ref(data)));
    threads.push_back(std::thread(rangeBaseToBogie, sim, std::ref(cv), std::ref(cv1),std::ref(friendly), std::ref(baseContainer), std::ref(data)));
    threads.push_back(std::thread(showData, sim, std::ref(cv1), std::ref(cv2), std::ref(baseContainer),std::ref(friendly), std::ref(timeFriend), std::ref(rangeFriend), std::ref(baseCompare), std::ref(friendCompare),std::ref(data)));
    
    //Join threads and begin!
    for(auto & t: threads){
        t.join();
    }

  return 0;
}