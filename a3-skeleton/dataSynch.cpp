#include <iostream>
#include <vector>
#include <deque>
#include "dataSynch.h"
#include "simulator.h"

#define BUFFER 10

DataSynch::DataSynch(){

}

double DataSynch::interpolate(long xL, long xR, double yL, double yR, double x, bool extrapolate){
    if ( !extrapolate ){       
        if ( x < xL ) yR = yL;
        if ( x > xR ) yL = yR;
    }
    double dydx = ( yR - yL ) / ( xR - xL );                                   
    return yL + dydx * ( x - xL );       
}

void DataSynch::getBaseData(RangeStamped baseToBogie){
    cv.notify_all();
    baseContainer.push_front(baseToBogie);
    cv1.notify_all();
}

void DataSynch::checkBaseSize(){
    if(baseContainer.size() == BUFFER){
        baseContainer.pop_back();
    }
}

void DataSynch::getFriendlyData(RangeStamped friendlyToBogie){
    friendlyContainer.push_front(friendlyToBogie);
}

void DataSynch::checkFriendSize(std::unique_lock<std::mutex>& locker){
    if(friendlyContainer.size() == BUFFER){            
        RangeStamped x = friendlyContainer.front();
        RangeStamped y = friendlyContainer.at(1);
    for(int i = y.timestamp; i <= x.timestamp; i++){
        timeFriend.push_back(i);
    }
    for(double j : timeFriend){
        rangeFriend.push_back(interpolate(y.timestamp, x.timestamp, y.range,x.range, j, true));
    }
    cv.wait(locker);
    cv2.notify_all();
    friendlyContainer.clear(); 
    }   
}

void DataSynch::showData(std::unique_lock<std::mutex>& locker){
    cv1.wait(locker);
    std::cout << "Base: " << baseContainer.front().timestamp << " : " << baseContainer.front().range << std::endl;
    baseCompare.push_back(baseContainer.front().timestamp);
    baseCompare.push_back(baseContainer.front().range);
    cv2.wait(locker);
    for(int i= 0; i <= timeFriend.size() -1; i++){
        std::cout << timeFriend[i] << " : " << rangeFriend[i] << std::endl;
        if(timeFriend[i] == baseCompare[0]){
            friendCompare.push_back(timeFriend[i]);
            friendCompare.push_back(rangeFriend[i]); 
        }
    }
    timeFriend.clear();
    rangeFriend.clear();
}

std::vector<long> DataSynch::getBaseCompare(){
    return baseCompare;
}

std::vector<long> DataSynch::getFriendCompare(){
    return friendCompare;
}

void DataSynch::resetData(){
    baseCompare.clear();
    friendCompare.clear();   
}

