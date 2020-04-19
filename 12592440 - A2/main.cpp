#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "laser.h"
#include "radar.h"
#include "rangerfusion.h"

int main(){
    //Initialising the sensors and variables to be used
    Laser laser;
    Radar radar1;
    Radar radar2;
    FusionMethod user;
    int userInput, laserAngResInput, laserAngRes, radar1Offset, radar2Offset;
    
    //Displays the default parameters of the Laser
    std::cout << "The default laser Angular Resolution is " << laser.getAngularResolution() << " degrees" << std::endl;
    std::cout << "The default laser offset is " << laser.getOffset() << " degrees" <<" (Fixed Parameter)" << std::endl;
    std::cout << "The default laser Field of View is " << laser.getFieldOfView() << " degrees" <<" (Fixed Parameter)" << std::endl;
    std::cout << "The Laser Maximum & Minimum distance is " << laser.getMaxRange() << "m & " << laser.getMinRange() << "m" << " (Fixed Parameters)" << std::endl;
    std::cout << "Enter 0 for 10 degrees or 1 for 30 degrees laser angular resolution:" << std::endl;
    std::cin >> laserAngResInput;

    //Asks User what angular resolution they want for the laser, if value is not sane will use default values instead
    switch(laserAngResInput){
        case 0:
            laserAngRes = laserAngResInput;
            break;
        case 1:
            laserAngRes = laserAngResInput;
            break;
        default:
            std::cout << "Input is out of the bounds, laser angular resolution will be default 10 degrees" << std::endl;
            laserAngRes = laser.getAngularResolution();   
            break;     
    }
    laser.setOffset(laserAngRes);
    
    //Asks user for the parameters of the radars, if value is not sane will use default values insteads
    std::cout << "Set the offset of the Radars from 120 to -120 degrees:" << std::endl;
    std::cout << "Enter Radar 1 offset: " ;
    std::cin >> radar1Offset;
    if(radar1Offset > 120 || radar1Offset < -120){
        std::cout << "Radar 1 is out of the boundaries, will be set to default offset of 0." << std::endl;
        radar1.setOffset(0); 
    }else{
        radar1.setOffset(radar1Offset);
    }

    std::cout << "Enter Radar 2 offset: " ;
    std::cin >> radar2Offset;
    if(radar2Offset > 120 || radar2Offset < -120){
        std::cout << "Radar 2 is out of the boundaries, will be set to default offset of 0." << std::endl;
        radar2.setOffset(0); 
    }else{
        radar2.setOffset(radar2Offset);
    }

    RangerFusion rangers;
    std::vector<RangerInterface*> holder = { &laser, &radar1, &radar2};  //sends the parameters of the sensors 
    rangers.setRangers(holder);

    //Asks user which Fusion Method they would like
    std::cout << "Default is FUSION_MIN" <<std::endl << "Enter 0 for FusionMin, 1 for FusionMax & 2 for FusionAvg:" << std::endl;
    std::cin >> userInput;
    if(userInput == 0){
        user = FUSION_MIN;
    } else if(userInput == 1){
        user = FUSION_MAX;
    } else if(userInput == 2){
        user = FUSION_AVG;
    }else{
        std::cout << "Entered Default Value, Program will proceed to FusionMin:" << std::endl;
        user = FUSION_MIN;
    }
    rangers.setFusionMethod(user);
    
    while(1){
        std::vector<std::vector<double>> data = rangers.getRawRangeData();  //obtains the Raw Data
        std::cout << std::setprecision(5);
        std::vector<std::vector<std::vector<double>>> sensorAngle = rangers.getAngles();    //obtains the angles for each sensor

        //Displays the Data and Angles for each sensor
        for(int i = 0; i < data.size(); i++){
            std::cout << std::endl;
            for(int r = 0; r < data[i].size(); r++){
                std::cout << std::left << std::setw(10) << data[i][r] << std::setw(10);
            }
            std::cout << std::endl;
            for(int r = 0; r < data[i].size(); r++){
                std::cout << std::left << std::setw(10) << sensorAngle[1][i][r];
            }

            std::cout << std::endl;
        }
        data.clear();   //Empties the data to repeat process and obtain new Data
        sensorAngle.clear();
        std::cout << std::endl;

        std::cout << "Fused Data: ";
        std::vector<double> fused = rangers.getFusedRangeData();    //obtains fusion data depending on fusion method chosen
        for(int i = 0; i < fused.size(); i++){
            std::cout << fused[i] << "  ";      //displays the fused data 
        }
        std::cout << std::endl;
    }
    return 0;

}