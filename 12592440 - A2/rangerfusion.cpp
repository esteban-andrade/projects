#include <vector>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <thread>
#include "rangerfusion.h"

const int anglesSection = 1;
const int readingSection = 0;

RangerFusion::RangerFusion(){
}

void RangerFusion::setRangers(std::vector<RangerInterface*> rangers){
    rangers_ = rangers; //sends the sensors objects to rangers_
}

std::vector<double> RangerFusion::getFusedRangeData(){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::vector<double> fusedData;  //create a temp vector which will be filtered for the fusion method then pushed to the fusedData vector
    std::vector<double> temp ;
    int sample = 0;
    double avg = 0;
    switch (input_){
               
        case FUSION_MIN:
            sample = 0;
            avg = 0;
            for(int angle = laserAngle_[0]; angle >= laserAngle_.back(); angle -= laserRes){
                temp.clear();   //clears the content of the temp vector
                temp.push_back(laserData_[sample]);     //pushes the laser data to first index
                for(int r = 1; r < rangers_.size(); r++){               //loops through the radars
                    for(int d = 0; d < radar0_.size(); d++){
                        if(angle - radar1_[r-1][d] >= -radarAngRes/2 && angle - radar1_[r-1][d] <= radarAngRes/2){  //checks if the laser angle is in the area of the radar
                        temp.push_back(radarNumber_[r-1][d]);   //if it is in the area of the angle the radar data gets pushed back
                        }
                    }
                }
                sample++;   //increments for the next data of laser
                fusedData.push_back(*std::min_element(temp.begin(), temp.end())); //pushes the fused data according to fusionMethod input which is FUSION_MIN 
            }
            return fusedData;
        
        case FUSION_MAX:
            for(int angle = laserAngle_[0]; angle >= laserAngle_.back(); angle -= laserRes){
                temp.clear();   //clears the content of the temp vector
                temp.push_back(laserData_[sample]);     //pushes the laser data to first index
                for(int r = 1; r < rangers_.size(); r++){            //loops through the radars
                    for(int d = 0; d < radar0_.size(); d++){
                        if(angle - radar1_[r-1][d] >= -radarAngRes/2 && angle - radar1_[r-1][d] <= radarAngRes/2){  //checks if the laser angle is in the area of the radar
                        temp.push_back(radarNumber_[r-1][d]);
                        }
                    }
                }
                sample++;   //increments for the next data of laser
                fusedData.push_back(*std::max_element(temp.begin(), temp.end()));   //pushes the fused data according to fusionMethod input which is FUSION_MIN 
            }

            return fusedData;

        case FUSION_AVG:
            for(int angle = laserAngle_[0]; angle >= laserAngle_.back(); angle -= laserRes){
                temp.clear();   //clears the content of the temp vector
                temp.push_back(laserData_[sample]);     //pushes the laser data to first index
                for(int r = 1; r < rangers_.size(); r++){             //loops through the radars
                    for(int d = 0; d < radar0_.size(); d++){
                        if(angle - radar1_[r-1][d] >= -radarAngRes/2 && angle - radar1_[r-1][d] <= radarAngRes/2){  //checks if the laser angle is in the area of the radar
                        temp.push_back(radarNumber_[r-1][d]);   
                        }
                    }
                }
                sample++;       //increments for the next data of laser
                for(int i = 0; i < temp.size(); i++){   //Goes thrpugh the size of the temp vector
                    avg += temp.at(i);      //Adds the value of each index to avg
                }
                avg = avg / temp.size();    //computes the average by the total avg and the temp.size
                fusedData.push_back(avg);   //pushes the fused data according to fusionMethod input which is FUSION_MIN 
                avg = 0;    //resets average for next reading
            }
            return fusedData;
    }    
}

std::vector<std::vector<double>> RangerFusion::getRawRangeData(){
    data_.clear();  //empties the data vector
    for(int i = 0; i < rangers_.size(); i ++){
        data_.push_back(rangers_[i]->generateData());      //obtains the random value to the sensors
    }
    angles_.push_back(data_);   //pushes the data to a 3d vector
    angles_.push_back(data_);   //pushes the data again to obtain angle of the sensor

    for(int i = 0; i < rangers_.size(); i++){
        int angRes = rangers_[i]->getAngularResolution();   //obtain parameters of the sensors
        int offset = rangers_[i]->getOffset();
        int FOV = rangers_[i]->getFieldOfView();
        for(int j = 0; j < data_[i].size(); j++){
            
            if(FOV == 180){     //if it is a laser, obtain the angles for it and data
                int angleAt = (FOV+offset)/2-(j*angRes) ;
                laserRes = rangers_[i]->getAngularResolution();
                angles_[anglesSection][i].at(j) = angleAt;
                if(laserAngle_.size() != FOV/angRes + 1){   //obtain readings until the number of samples
                    laserAngle_.push_back(angleAt);
                    laserData_.push_back(angles_[readingSection][i][j]);
                }
                
            }
            if(FOV == 60){  //if it is a radar, obtain the angles, data and how many radars there are
                int angleAt = (FOV+offset)/2-(j*angRes) - angRes/2;
                angles_[anglesSection][i].at(j) = angleAt;
                radarOffset = rangers_[i]->getOffset();
                radarAngRes = rangers_[i]->getAngularResolution();
                if(radar0_.size() != FOV/angRes){   //obtain readings until the number of samples
                    radar0_.push_back(angleAt);
                    radarData_.push_back(data_[i][j]);
                }else{
                    radarNumber_.push_back(radarData_);
                    radar1_.push_back(radar0_);
                    radarData_.clear();
                    radar0_.clear();
                    radarData_.push_back(data_[i][j]);
                    radar0_.push_back(angleAt);                    
                }
            }
        }
    }
    radarNumber_.push_back(radarData_);
    radar1_.push_back(radar0_); 

    return(data_);
}

void RangerFusion::setFusionMethod(FusionMethod userInput){
    input_ = userInput;     //to check what fusionMethod is to be used
}

std::vector<std::vector<std::vector<double>>> RangerFusion::getAngles(){
    return angles_;     //used to display the angles on the screen
}