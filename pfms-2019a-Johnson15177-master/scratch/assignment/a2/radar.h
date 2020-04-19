#ifndef RADAR_H
#define RADAR_H

#include <string>
#include "ranger.h"
#include "rangerinterface.h"
//#include "numgenerator.h"

using namespace std;

class Radar: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
    Radar();
   
    std::vector<double> generateData();
  
    unsigned int getAngularResolution(void);
    int getOffset(void);
    unsigned int getFieldOfView(void);
    int getBaud(void);
    string getModel(void);
    
    int setAngularResolution(int);
    int setOffset(int);
    int setFieldOfView(unsigned int);
    int setBaud(int);
   
  
    double getMinDistance(void);
    double getMaxDistance(void);

private:
    string model;
    int baud;
    int offSet;
    int FOV;
    int angRes;
    double maxDistance;
    double minDistance;  
    
    int ang_res1_ = 20;
    int FOV_ = 60;
    string model_ = "RAD-001";
    int baud1_ = 38400;
    int baud2_ = 115200;
    double max_distance_ = 16.0;
    double min_distance_ = 0.2;
    int offSet_;
};

#endif // RADAR_H
