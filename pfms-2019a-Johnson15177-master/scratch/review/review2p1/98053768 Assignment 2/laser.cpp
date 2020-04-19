#include <string>
#include <vector>
#include "laser.h"

// Constructor sets default values
Laser::Laser () {
    // Set laser default values
    model_ = "UTM-XXL";
    field_of_view_ = 180;
    angular_res_ = 30;
    offset_ = 0;
    min_range_ = 0.2;
    max_range_ = 8.0;
    current_angle_ = 0;
    sample_num_ = 0;

    // Recalculate number of samples based on
    // new field of view & angular resolution
    // and the fact that the laser has one extra
    // reading over its field of view
    total_readings_ = field_of_view_/angular_res_ + 1;
}

bool Laser::setAngularResolution(unsigned int ang_res) {
    // Checks if angular resolution is 10 or 30 degrees
    if (ang_res == 10 || ang_res == 30) {
        // Set specified angular resolution
        angular_res_ = ang_res;

        // Recalculate number of samples based on
        // new angular resolution
        total_readings_ = field_of_view_/angular_res_ + 1;

        return true;
    }
    else {
        // Make sure angular resolution is set to default value of 30
        angular_res_ = 30;

        // Recalculate number of samples based on
        // new angular resolution
        total_readings_ = field_of_view_/angular_res_ + 1;

        return false;
    }
}

bool Laser::setOffset(int offset) {
    /* Offset of laser is fixed at zero
     * All this function does is tell the user if the input
     * is 0 degrees or not
     */
    if (offset == 0) {
        return true;
    }
    else {
        return false;
    }
}

bool Laser::setFieldOfView(unsigned int fov) {
    /* Field of view can only be 180 degrees and
     * is already set to 180 degrees in the constructor.
     * All this function does is tell the user if the input
     * is 180 degrees or not
     */
    if (fov == 180) {
        return true;
    }
    else {
        return false;
    }
}

