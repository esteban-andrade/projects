#include "ranger.h"
#include "rangerfusion.h"
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "radar.h"
#include "laser.h"
#include "thread"

#include <iostream>
using namespace std;
string a;
int method;
int baud;
unsigned int angularRes;
void printLaserInfo(vector<Laser*> &LaserVec){
    for(auto it = LaserVec.begin(); it != LaserVec.end(); it++){
        cout << "Model Name: "<< (*it)->getModel() << endl;
        cout << "Field of View: " <<(*it)-> getFieldOfView() << endl;
        cout << "Max Distance: " << (*it)->getMaxRange() << endl;
        cout << "Min Distance: " << (*it)->getMinRange() << endl;
        cout << "Angular Resolution: " <<(*it)-> getAngularResolution() << endl;
        cout << "Orientation Offset: " <<(*it)-> getOffset() << endl;
        cout << "-----------------------------------------------------------" << endl;
    }
}

void printRadarInfo(vector<Radar*> &RadarVec){
    for(auto it = RadarVec.begin(); it != RadarVec.end(); it++){
        cout << "Model Name: "<< (*it)->getModel() << endl;
        cout << "Field of View: " <<(*it)-> getFieldOfView() << endl;
        cout << "Max Distance: " << (*it)->getMaxRange() << endl;
        cout << "Min Distance: " << (*it)->getMinRange() << endl;
        cout << "Angular Resolution: " <<(*it)-> getAngularResolution() << endl;
        cout << "Orientation Offset: " <<(*it)-> getOffset() << endl;
        cout << "-----------------------------------------------------------" << endl;
    }
}



int main(){
    // create object of laser and radar
    Laser laser1;
    Radar radar1;
    Radar radar2;
    // create a laser vector and store the laser object into it
    vector<Laser*> LaserVec;
    LaserVec.push_back(&laser1);

    // create a radar vector and store the laser object into it
    vector<Radar*> RadarVec;
    RadarVec.push_back(&radar1);
    RadarVec.push_back(&radar2);
    //Printing Default setting of all sensors object
    printLaserInfo(LaserVec);
    printRadarInfo(RadarVec);


    //Ask if user want to change the setting of all sensors
    cout << "Do you want to change the setting of all sensors ? (y for yes, n for no)" << endl;
    cin >> a;
    if(a == "y")
    {
        cout << "Setting the new laser value :" << endl;
        cout << "Please enter the value for laser angular resolution" << endl;
        cin >> angularRes;
        if(angularRes == 10)
        {
            laser1.setAngularResolution(angularRes);
            int offsetA = 40;
            int offsetB = 100;
            radar1.setOffset(offsetA);
            radar2.setOffset(offsetB);
        }
        else if(angularRes == 30){
            laser1.setAngularResolution(angularRes);
            int offsetC = 70;
            radar1.setOffset(offsetC);
            radar2.setOffset(offsetC);
        }
        else
        {
            cout << "Incorrect input ! The angular resolution and offset has been set to default value:" << laser1.getAngularResolution() << endl;
        }
    }

    //create object fusion
    RangerFusion fusion;

    cout << "type for fusion method(0 for min, 1 for max, 2 for average" << endl;
    cin >> method;
    if(method == 0 || method == 1 || method == 2)
    {
        fusion.setFusionMethod(method);
    }
    else
    {
        method = 0;
        fusion.setFusionMethod(method);
        cout << "invail input, method has been set to default" << endl;
    }


    while(1){

        // loop all the initialization
        std::this_thread::sleep_for(std::chrono::seconds(1));
        laser1.getNumOfSamples();
        laser1.putDataVec();
        radar1.getNumOfSamples();
        radar1.putDataVec();
        radar2.getNumOfSamples();
        radar2.putDataVec();

        fusion.getLaserObject(laser1);
        fusion.getRadarObject(radar1);
        fusion.getRadarObject(radar2);

        fusion.fusion();
        fusion.setFuseRangeData();

        //print angle and fuse data
        for(int i=0;  i<fusion.getFusedRangeData().size() ; i++){
            cout <<"Angle "<< i*laser1.getAngularResolution() << ":"<< fusion.getFusedRangeData().at(i) << endl;
        }
        cout << "-------------------------------" << endl;


        //clear all the container
        laser1.clear_value();
        radar1.clear_value();
        radar2.clear_value();
        fusion.clear();
    }

    //create containner for rangers
    std::vector<RangerInterface*> rangers;
    //store rangers sensors into rangers
    rangers = {&laser1, &radar1, &radar2};
    fusion.setRangers(rangers);
    return 0;
}
