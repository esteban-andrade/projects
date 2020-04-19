#include "rangerfusion.h"
RangerFusion::RangerFusion()
{

}

void RangerFusion::setRangers(std::vector<RangerInterface *> &rangers)
{
    rangers_ = rangers;

}

std::vector<RangerInterface *> RangerFusion::getRangers()
{
    return rangers_;
}

void RangerFusion::getLaserData(vector<double> lasers)
{
    laser_data_.push_back(lasers);
}

void RangerFusion::getRadarData(vector<double> radars)
{
    radar_data_.push_back(radars);
}

void RangerFusion::getLaserObject(Laser &object)
{
    laser_object_.push_back(object);
}

void RangerFusion::getRadarObject(Radar &object)
{
    radar_object_.push_back(object);
}

void RangerFusion::clear()
{
    laser_object_.clear();
    radar_object_.clear();
    laser_data_.clear();
    radar_data_.clear();
    fuseData.clear();
}




void RangerFusion::fusion()
{

    vector<double> triangle;      // vector for calculate radar triangle
    vector<double> tempFuse;     // temper vector to for fusion process
    maps.clear();
    tempFuse.clear();

    // triangle equals to offset - 10 degree
    for(int a = 0; a < radar_object_.size(); a++)
    {
        triangle.push_back(radar_object_.at(a).getOffset() - 10);
    }

    //get laser object reading, depends on the size of laser object
    for(int b = 0; b < laser_object_.size(); b++)
    {
        getLaserData(laser_object_.at(b).generateData());
    }
    //get radar object reading, depends on the size of radar object
    for(int c = 0; c < radar_object_.size(); c++)
    {
        getRadarData(radar_object_.at(c).generateData());
    }

    // get the laser angular_resolution
    int angular_res =  laser_object_.at(0).getAngularResolution();
    // loop size of laser reading 19 times

    for(int i = 0; i < laser_data_.at(0).size();i++)
    {
        // put all laser value into tempfuse for more than one laser
        for(int ia = 0; ia < laser_data_.size(); ia++)
        {
            tempFuse.push_back(laser_data_.at(ia).at(i));
        }


        // put all radar value into tempfuse for more than one radar
            for(int ib = 0; ib < radar_data_.size(); ib++)
            {
                //push radar radar return reading in the first triangle into tempfuse vector
                if(triangle.at(ib) <= i*angular_res && triangle.at(ib) + 20 >= i*angular_res )
                {
                    tempFuse.push_back(radar_data_.at(ib).at(0));
                }

                // push radar return reading in the second triangle into tempfuse vector
                if(triangle.at(ib) + 20 <= i*angular_res && triangle.at(ib) + 40 >= i*angular_res )
                {
                    tempFuse.push_back(radar_data_.at(ib).at(1));
                }


                // push radar return reading in the third triangle into tempfuse vector
                if(triangle.at(ib) + 40 <= i*angular_res  && triangle.at(ib) + 60 >= i*angular_res)
                {
                    tempFuse.push_back(radar_data_.at(ib).at(2));
                }
            }


        // if user choose method is 0, find the min value ,
        if(method_ == FUSION_MIN)
        {
            double min = 16;
            for(int x = 0; x < tempFuse.size(); x++)
            {
                if(min > tempFuse.at(x))
                {
                    min = tempFuse.at(x);
                }
            }
            maps[angular_res*i] = min;
            tempFuse.clear();
        }

        //find max
        if(method_ == FUSION_MAX)
        {
            double max = 0;
            for(int x = 0; x < tempFuse.size(); x++)
            {
                if(max < tempFuse.at(x))
                {
                    max = tempFuse.at(x);
                }
            }
            maps[angular_res*i] = max;
            tempFuse.clear();
        }


        if( method_ == FUSION_AVG)
        {
            //find average reading , add up readings and devide.
            double sum = 0;

            for (auto x: tempFuse)
            {
                sum = sum + x;
            }
            auto x =tempFuse.size();
            maps[angular_res*i] = sum/x;

            tempFuse.clear(); //clear vector
        }
    }




}

void RangerFusion::setFuseRangeData()
{
    for(auto it = maps.begin(); it!= maps.end(); it++)
    {
         fuseData.push_back(it->second);
    }

}


std::vector<double> RangerFusion::getFusedRangeData()
{

    return fuseData;
}


std::vector<std::vector<double> > RangerFusion::getRawRangeData()
{
    for(int i = 0; i < rangers_.size(); i++ )
    {
        data_.push_back(rangers_.at(i)->generateData());
    }

    return data_;
}


void RangerFusion::setFusionMethod(int &method)
{
    if(method == 0)
    {
        method_ == FUSION_MIN;
    }

    else if(method == 1)
    {
        method_ == FUSION_MAX;
    }

    else if(method == 2)
    {
        method_ == FUSION_AVG;
    }

}

