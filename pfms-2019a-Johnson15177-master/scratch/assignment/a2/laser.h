#ifndef LASER_H
#define LASER_H

#include <string>
#include "ranger.h"
//#include "numgenerator.h"


using namespace std;

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
    Laser();
  
   // std::vector<double> generateData();
  
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
    string model_ = "UTM-XXL";
    
    int FOV;
    int FOV_ = 180;
    double maxDistance;
    double minDistance;
    
    double max_distance_ = 8.0;
    double min_distance_ = 0.2;
    
    int baud;
    int baud1_ = 38400;
    int baud2_ = 115200;
    
    int angRes;
    int ang_res1_ = 10;
    int ang_res2_ = 30;
    
    int offSet;
    int offSet_ = 0; 
};

#endif // LASER_H
