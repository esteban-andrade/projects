#include "rangerfusion.h"
#include "rangerinterface.h"

#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <stdlib.h>

RangerFusion::RangerFusion()
{
    fusion_method_ = FUSION_MIN;
}
/* sets the vector of sensor objects */
void RangerFusion::setRangers(std::vector<RangerInterface*> rangers)
{
    rangers_ = rangers;
}
/* function gets raw data and fills each sensor  */
std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{
    /* makes the program stop for a second as if it is at a baud rate of 1hz */
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::vector<double>> vectors(rangers_.size());
    /* cycles through the sensor objects and generates the data */
    for (int i=0; i<rangers_.size(); i++)
    {
        vectors.at(i) = rangers_.at(i)->generateData();
    }
    data_ = vectors;
    /* makes fused data = the same data at laser and will be replaced buy the fused data later */
    fused = data_.at(0);

    return (vectors);
    
}
/* gets the fused data */
std::vector<double> RangerFusion::getFusedRangeData()
{
    return fused;
}
/* allows input for choosing fusion method */
bool RangerFusion::takeInput(unsigned int input)
{
    if (input == FUSION_MIN)
    {
        fusion_method_= FUSION_MIN;
        return true;
    }
    else if (input == FUSION_MAX)
    {
        fusion_method_= FUSION_MAX;
        return true;
    }
    else if (input == FUSION_AVG)
    {
        fusion_method_= FUSION_AVG;
        return true;
    }
    else return false;
}
/* compares the laser index with each radar index until it satisfies the equation to allow for fusion of data */
void RangerFusion::setFusionMethod(FusionMethod)
{
    /* creates laser index at 19 or 7 depending on angular resolution */
    laser_index_ = (rangers_.at(0)->getFieldOfView())/(rangers_.at(0)->getAngularResolution()) +1;
    /* creates radar index at 3 */
    radar_index_ = (rangers_.at(1)->getFieldOfView())/(rangers_.at(1)->getAngularResolution());
/* compares the laser data with each of the radar data, if it satisfies the equation it does the fusion min, max or avg */
    for(int i=0; i<laser_index_; i++)
    {
        for(int j=1; j<=2; j++)
        {
            for(int k=0; k<radar_index_; k++)
            {
                /* equation compares laser and radar data positions and checks if it is within 10 degrees of each other, if yes continues to the method */
                if (abs((rangers_.at(0)->getAngularResolution() * i) - (rangers_.at(1)->getAngularResolution() * k) + rangers_.at(j)->getOffset() -70) <= 10)
                {
                    /* min method checks if radar is less than laser data if yes replaces the value at that index with the radar value */
                    if (fusion_method_== FUSION_MIN)
                    {
                        if (data_.at(j).at(k) < fused.at(i))
                        {
                            fused.at(i) = data_.at(j).at(k);
                        }
                    }
                    /* max method checks if radar is greater than laser data if yes replaces the value at that index with the radar value */
                    else if (fusion_method_ == FUSION_MAX)
                    {

                        if(data_.at(j).at(k) > fused.at(i))
                        {
                            fused.at(i) = data_.at(j).at(k);
                        }
                    }
                }
            }
        }
    }

    double sum=0;
    double counter=1;
    double total=0;
/* compares the laser index with the radar index for both radars before incrementing the radar index */
    for(int i=0; i<laser_index_; i++)
    {
        /* adds the current laser data we are looking at into the sum container */
        sum += fused.at(i);
        for(int k=0; k<radar_index_; k++)
        {
            for(int j=1; j<=2; j++)
            {
                /* equation compares lasar and radar data positions and checks if it is within 10 degrees of each other, if yes continues to the method*/
                if (abs((rangers_.at(0)->getAngularResolution() * i) - (rangers_.at(1)->getAngularResolution() * k) + rangers_.at(j)->getOffset() -70) <= 10)
                {
                    /* avg method takes data and returns the average to the fused vector */
                    if (fusion_method_ == FUSION_AVG)
                    {
                        /* adds value to sum and increments counter */
                        sum += data_.at(j).at(k);
                        counter++;
                    }
                }

            }
            /* completes the averageing calculations, if no values added to sum skips */
            if(counter!=1)
            {
                total = sum / counter;
                fused.at(i) = total;
                total=0;
                counter=1;

            }
        }
        /* makes sum = 0 whether used or not */
        sum = 0;
    }

}
