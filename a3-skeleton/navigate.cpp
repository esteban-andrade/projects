#include <cmath>
#include <deque>
#include "lib/simulator.h"
#include "navigate.h"

Navigate::Navigate(){

}

void Navigate::getVectorOrientation(std::deque<Pose> correctBogie, Pose friendly){
    vectorOrientation = atan2(correctBogie.front().position.y - friendly.position.y, correctBogie.front().position.x - friendly.position.x);
    if(vectorOrientation <= 0)
        vectorOrientation = 2*M_PI - fabs(vectorOrientation); 
    offset =  vectorOrientation - friendly.orientation;
    std::cout << "vectorOrientation: " << vectorOrientation << std::endl;
    std::cout << "offset: " << offset << std::endl;
    double y = fabs(offset);
    double x = 2*M_PI - y;
}

void Navigate::move(){
    if(offset < 0.087 && offset > -0.087){
        linSpeed = 1000;
        angVel = 0;
    }else if (offset < 0){
        linSpeed = 50;
        angVel = -1.1772;
    }else if( offset > 0){
        linSpeed = 50;
        angVel = 1.1772;
    } 
}

void Navigate::checkAirspace(const std::shared_ptr<Simulator> & sim ,Pose friendly){
    if(friendly.position.x > sim->AIRSPACE_SIZE || friendly.position.y > sim->AIRSPACE_SIZE){
        double orientation = friendly.orientation;  
        while(orientation == -orientation){
            angVel = 1.1772;
            linSpeed = 50;
        }
        angVel = 0;
        linSpeed = 1000;
    }
}

double Navigate::getLinSpeed(){
    return linSpeed;
}

double Navigate::getAngVel(){
    return angVel;
}
