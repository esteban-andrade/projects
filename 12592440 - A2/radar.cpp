#include <iostream>
#include "radar.h"
#include "ranger.h"

int radarOffset_ = 0; //Orientation offset
const int radarAngleRes_ = 20;
const double radarMaxDistance_ = 16.0;
const double radarMinDistance_ = 0.2;
const double radarFieldOfView_ = 60;
const int radarNumSamples_ = radarFieldOfView_/radarAngleRes_ ; 

Radar::Radar():Ranger(radarAngleRes_, radarOffset_, radarFieldOfView_, 
    radarMaxDistance_, radarMinDistance_, radarNumSamples_){
}
