#include <vector>
#include "rangerfusion.h"
#include "rangerfusioninterface.h"
#include "rangerinterface.h"

//Default constructor should set all RangerFusion attributes to a default value
RangerFusion::RangerFusion() :
    fusion_method_(FUSION_AVG),
    sample_num_(1),
    laser_index_(0)
{

}

// Returns current sample number
int RangerFusion::getSampleNum(void) {
    return sample_num_;
}

// Set sample number
bool RangerFusion::setSampleNum(int sample_num) {
    if (sample_num > 0) {
        sample_num_ = sample_num;
        return true;
    }
    else {
        sample_num_ = 1;
        return false;
    }
}

// Accepts container of rangers - as per requirement C2
void RangerFusion::setRangers(std::vector<RangerInterface*> rangers) {
    rangers_ = rangers;
}

// Returns a container of fused internal data readings - as per requirement C5
std::vector<double> RangerFusion::getFusedRangeData() {
    std::vector<double> laser_data;         // Stores laser data
    std::vector<double> fused_data;         // Stores fused data
    std::vector<double> temp_radar_data;    // Temporary storage for radar data

    /* Procedure:
     * 1. Determine which sensor is the laser.
     * 2. Go through each of the laser's readings using the offset, field of view and resolution.
     * 3. At each laser's reading, go through each radar reading and use the offset and
     *    field of view to see if the radar has a reading that coincides with the current
     *    laser reading.
     * 4. If a radar has a reading that coincides with the laser reading, fuse the reading
     *    with the laser reading according to the specified method. Repeat this for all radar
     *    readings
     */

    // STEP 1:
    // Find the largest vector of data. This has come from the laser
    laser_data = data_.at(laser_index_);
    for (int i = 0; i<data_.size(); i++) {
        temp_radar_data = data_.at(i);
        if(temp_radar_data.size() > laser_data.size()) laser_index_ = i;
    }

    // Store laser data
    laser_data = data_.at(laser_index_);

    // Ensure fused data container is large enough to store all the values
    fused_data.reserve(laser_data.size());


    // STEP 2:
    // Get the starting and ending angles of the laser using offset and field of view
    int start_laser_angle = rangers_.at(laser_index_)->getOffset();
    start_laser_angle -= rangers_.at(laser_index_)->getFieldOfView()/(2);
    int end_laser_angle = start_laser_angle + rangers_.at(laser_index_)->getFieldOfView();


    // Initialise temporary variables
    int laser_counter = 0;          // Counter for laser data
    double temp_radar;              // Temporary storage for a radar data value
    double temp_fused;              // Temporary storage for a fused data value
    double total_fused;             // Stores the total value of all the readings at one position (average method only)
    double num_fused = 1;           // Stores how many data values are fused (average method only)
    int radar_current_angle_min;    // Stores min angle the value in temp_radar is measured over
    int radar_current_angle_max;    // Stores max angle the value in temp_radar is measured over

    // Go through each data reading of the laser
    for (int current_laser_angle = start_laser_angle;
             current_laser_angle <= end_laser_angle;
             current_laser_angle += rangers_.at(laser_index_)->getAngularResolution())
    {
        // Collect the laser data value at the start of the reading
        temp_fused = laser_data.at(laser_counter);;
        total_fused = temp_fused;
        num_fused = 1;

        // STEP 3:
        // Go through the radar data values and check if they correspond to the laser angle
        for (int i = 0; i < data_.size(); i++) {
            if (i != laser_index_) {
                // Get data for the current radar
                temp_radar_data = data_.at(i);

                // Go through each radar value to see if their angle corresponds to the
                // current laser angle
                for (int j = 0; j < temp_radar_data.size(); j++) {
                    temp_radar = temp_radar_data.at(j);
                    radar_current_angle_min = rangers_.at(i)->getOffset();
                    radar_current_angle_min -= rangers_.at(i)->getFieldOfView()/2;
                    radar_current_angle_min +=rangers_.at(i)->getAngularResolution()*j;
                    radar_current_angle_max = radar_current_angle_min + rangers_.at(i)->getAngularResolution();

                    // STEP 4:
                    // If the angles correspond, fuse the data according to the selected fusion method
                    if (current_laser_angle >= radar_current_angle_min &&
                        current_laser_angle <= radar_current_angle_max)
                    {
                        switch(fusion_method_) {
                            case FUSION_MIN:
                                // Replace fused value if radar value is smaller
                                if (temp_radar < temp_fused) temp_fused = temp_radar;
                                break;
                            case FUSION_MAX:
                                // Replace fused value if radar value is larger
                                if (temp_radar > temp_fused) temp_fused = temp_radar;
                                break;
                            case FUSION_AVG:
                                // Update average value
                                num_fused++;
                                total_fused = (total_fused + temp_radar);
                                temp_fused = total_fused/num_fused;
                                break;
                        }
                    }
                }
            }
        }
        // Once all the radar values have been processed, store the fused data value
        fused_data.push_back(temp_fused);
        laser_counter++;
    }
    sample_num_++;
    return fused_data;
}

// Returns a container of raw data range readings - as per requirement C4
std::vector<std::vector<double>> RangerFusion::getRawRangeData() {
    // Clear the data vector
    data_.clear();

    // Make sure data vector is large enough to store data from all the rangers
    data_.reserve(rangers_.size());

    // Populate raw data and return
    for (int i = 0; i < rangers_.size(); i++) {
        data_.push_back(rangers_.at(i)->generateData());

        /* If the current object is a laser, remember the index for fusion.
         * This uses dynamic_cast to check if the current pointer is pointing
         * to a laser object.
         * As such, there is a coupling between the RangerFusion and Laser classes,
         * but the specification specifies that the data is fused at the resolution
         * of the laser.
         * NOTE this method needs the laser.h file to be included and is currently unused
         * but left here for future reference
         */
//        if (dynamic_cast<Laser*>(rangers_.at(i))) laser_index_ = i;
    }
    return data_;
}

// Sets the fusion method of the RangerFusion class - as per requirement C6
void RangerFusion::setFusionMethod(FusionMethod fusion_method) {
    // Checks if selected fusion method is valid
    switch (fusion_method) {
        case FUSION_MIN:
            fusion_method_ = fusion_method;
            break;
        case FUSION_MAX:
            fusion_method_ = fusion_method;
            break;
        case FUSION_AVG:
            fusion_method_ = fusion_method;
            break;
        default:
            // If the fusion method isn't valid, set it to default of avg
            fusion_method_ = FUSION_AVG;
            break;
    }
}

FusionMethod RangerFusion::getFusionMethod(void) {
    return fusion_method_;
}
