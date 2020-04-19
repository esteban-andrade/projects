#include <string>
#include <vector>
#include "radar.h"

Radar::Radar () {
    // Set radar default values
    model_ = "RAD-001";
    field_of_view_ = 60;
    angular_res_ = 20;
    offset_ = 0;
    min_range_ = 0.2;
    max_range_ = 16.0;
    current_angle_ = 0;
    sample_num_ = 0;

    // Recalculate number of samples based on
    // new field of view & angular resolution
    // and the fact that the radar has one reading
    // per angular resolution
    total_readings_ = field_of_view_/angular_res_;
}

//----------------------------------SETTERS----------------------------------//

bool Radar::setAngularResolution(unsigned int ang_res) {
    /* Angular resolution can only be 20 degrees
     * and is already set to 20 degrees in the
     * constructor.
     * All this function does is tell the user if the input
     * is 20 degrees or not
     */
    if (ang_res == 20) {
        return true;
    }
    else {
        return false;
    }
}

bool Radar::setOffset(int offset) {
    // Offset can be from -120 to +120 deg
    if (offset <= 120 && offset >= -120) {
        offset_ = offset;
        return true;
    }
    else {
        offset_ = 0;
        return false;
    }
}

bool Radar::setFieldOfView(unsigned int fov) {
    /* Field of view can only be 60 degrees and
     * is already set to 60 degrees in the constructor.
     * All this function does is tell the user if the input
     * is 60 degrees or not
     */
    if (fov == 60) {
        return true;
    }
    else {
        return false;
    }
}
