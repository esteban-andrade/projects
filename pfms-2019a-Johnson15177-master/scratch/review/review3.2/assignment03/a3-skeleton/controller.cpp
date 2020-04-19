#include "controller.h"

#include <mutex>
#include <vector>
#include <deque>
#include <cmath>

// Constructor for controller object used to configure all member variables with default values
Controller::Controller(const std::shared_ptr<Simulator> & simulator_passed) :
    sim_(simulator_passed), currently_chasing_target_(default_target), friendly_lin_velocity_(sim_->V_TERM),
    friendly_ang_velocity_(default_ang_velocity), out_of_airspace_(default_out_of_airspace),
    maximum_g_force_(default_magnitude_of_max*sim_->MAX_G),
    reading_values_(default_reading_values), calculate_control_(default_calculate_control)
{
}


void Controller::checkTargetToChase(std::vector<Pose> & temp_bogey_vector, std::vector<Pose> & previous_bogey_pose, double & average_time_interval){
    bool changed_target = false;

    // Iterate through each calculated potential location of the bogey aircraft
    for (int i = 0; i < temp_bogey_vector.size(); i++){
        // If the position returned a nan value, set the target point to the previous, real position
        if(std::isnan(temp_bogey_vector.at(i).position.x)){
            temp_bogey_vector.at(i).position = previous_bogey_pose.at(i).position;
        }
    }

    // If target has not been changed yet (it should never have been)
    if((changed_target == false)){
        if(out_of_airspace_ == false){
            // Check distance between first calculated position of the bogey to make sure it still resides within the airspace
            if((sim_->distance(temp_bogey_vector.at(currently_chasing_target_).position, sim_->BSTATION_LOC) > ((sim_->AIRSPACE_SIZE)/2))){
                // If outside the airspace and we are currently chasing that point of intersection
                currently_chasing_target_ = !currently_chasing_target_;
                changed_target = true;

            }
            // Check distance between second calculated position of the bogey to make sure it still resides within the airspace
            if(((sim_->distance(temp_bogey_vector.at(!currently_chasing_target_).position, sim_->BSTATION_LOC)) > ((sim_->AIRSPACE_SIZE)/2))){
                // If the target has already been changed, both positions lay outside the airspace
                if(changed_target == true){

                    // Tell the friendly aircraft to return to the base station
                    temp_bogey_vector.at(currently_chasing_target_).position = {sim_->BSTATION_LOC};
                    temp_bogey_vector.at(!currently_chasing_target_).position = {sim_->BSTATION_LOC};

                    currently_chasing_target_ = 0;

                    out_of_airspace_ = true;

                } else{
                    // If the target hasn't already been changed, and we are chasing the second calculated target position
                    currently_chasing_target_ = !currently_chasing_target_;
                    changed_target = true;

                }
            }
        } else{
            // If the bogey plane was already out of the airspace and has come back in, reset flags and start chasing target positions again
            if((sim_->distance(temp_bogey_vector.at(currently_chasing_target_).position, sim_->BSTATION_LOC) <= ((sim_->AIRSPACE_SIZE)/2))){
                changed_target = true;
                out_of_airspace_ = false;
            } else if(((sim_->distance(temp_bogey_vector.at(!currently_chasing_target_).position, sim_->BSTATION_LOC)) <= ((sim_->AIRSPACE_SIZE)/2))){
                if(changed_target == false){
                    currently_chasing_target_ = !currently_chasing_target_;
                    changed_target = true;
                    out_of_airspace_ = false;
                }

            } else{
                // Otherwise if it is still outside the airspace, continue to hover around base station
                temp_bogey_vector.at(currently_chasing_target_).position = {sim_->BSTATION_LOC};
                temp_bogey_vector.at(!currently_chasing_target_).position = {sim_->BSTATION_LOC};

                currently_chasing_target_ = 0;
            }
        }
    }

    // If the target hasn't been changed yet (i.e. bogey plane is in the airspace)
    if((changed_target == false) && (out_of_airspace_ == false)){
        // If the difference in distance from friendly to the bogey is increasing, the plane is chasing the target away from the bogey; hence switch which target to chase
        if((std::fabs(last_friendly_reading_.range) > std::fabs(second_last_friendly_reading_.range*MF_friendly_radar)) && (std::fabs(last_friendly_reading_.range) >= check_target_range_boundary)){
            currently_chasing_target_ = !currently_chasing_target_;
            changed_target = true;
        }

    }

    // If target still hasn't been changed yet
    if((changed_target == false) && (out_of_airspace_ == false)){
        // If the target currently being chased travels a distance greater than physically possible by the plane, the friendly had crossed over the midpoint, so switch and start chasing the other intersect point (this will be the same physical point in reality)
        if(((sim_->distance(temp_bogey_vector.at(currently_chasing_target_).position, previous_bogey_pose.at(currently_chasing_target_).position)) > (MF_distance_between_targets*(sim_->MAX_V)*((double)(average_time_interval/1000)))) && (std::fabs(last_friendly_reading_.range) >= check_target_range_boundary)){
            currently_chasing_target_ = !currently_chasing_target_;
            changed_target = true;
        }
     }

}



void Controller::solveSimultaneousEquations(double & current_time, Pose & target_bogey_1, Pose & target_bogey_2, RangeStamped & distance_base_bogey){
    double distance_friendly_bogey;

    // If the current time is greater than the last timestamp obtained from the friendly radar, extrapolate to calculate the distance from the friendly to the bogey
    if (current_time > last_friendly_reading_.timestamp){
        distance_friendly_bogey = second_last_friendly_reading_.range + ((current_time - second_last_friendly_reading_.timestamp)/(last_friendly_reading_.timestamp - second_last_friendly_reading_.timestamp))*(last_friendly_reading_.range - second_last_friendly_reading_.range);
    } else {
        // Otherwise, interpolate
        distance_friendly_bogey = second_last_friendly_reading_.range + (((last_friendly_reading_.range - second_last_friendly_reading_.range)/(last_friendly_reading_.timestamp - second_last_friendly_reading_.timestamp))*(current_time - second_last_friendly_reading_.timestamp));
    }

    // Set up variables to be used in the quadratic formula
    double f = (((std::pow(friendly_pose_.position.x,2))+(std::pow(friendly_pose_.position.y,2))+(std::pow(distance_base_bogey.range,2))-(std::pow(distance_friendly_bogey,2)))/2);

    double a = ((std::pow(friendly_pose_.position.x, 2))+(std::pow(friendly_pose_.position.y, 2)));

    double b = (-2*f*friendly_pose_.position.x);

    double c = ((std::pow(f, 2))-((std::pow(friendly_pose_.position.y,2))*(std::pow(distance_base_bogey.range,2))));

    // Solve for the two intersection points of the circles and store in Pose variables to be used later
    target_bogey_1 = {(((-1*b)+(std::sqrt((std::pow(b,2))-(4*a*c))))/(2*a)), ((f-((((-1*b)+(std::sqrt((std::pow(b,2))-(4*a*c))))/(2*a))*friendly_pose_.position.x))/(friendly_pose_.position.y))};
    target_bogey_2 = {(((-1*b)-(std::sqrt((std::pow(b,2))-(4*a*c))))/(2*a)), ((f-((((-1*b)-(std::sqrt((std::pow(b,2))-(4*a*c))))/(2*a))*friendly_pose_.position.x))/(friendly_pose_.position.y))};

}



double Controller::calculateOrientation(Pose & Pose1, Pose & Pose2){

    // Calculate the difference in x and y position between the two poses
    double delta_x = Pose1.position.x - Pose2.position.x;
    double delta_y = Pose1.position.y - Pose2.position.y;

    double orientation = 0;

    // If angle lies on the y axis
    if(delta_x == 0){
        // If y is positive, angle is 90 degrees
        if(delta_y > 0){
            orientation = (0.5 * M_PI);
        } else{
            // If y is negative, angle is 270 degrees
            orientation = (1.5 * M_PI);
        }
    } else if(delta_x > 0){
        // If angle lies in the first quadrant, solve using simple trig
        if(delta_y > 0){
            orientation = (std::atan((std::fabs(delta_y)/std::fabs(delta_x))));
        } else if(delta_y < 0){
            // If angle lies in 4th quadrant, subtract calculated angle from 360 degrees
            orientation = ((2*M_PI) - std::atan((std::fabs(delta_y)/std::fabs(delta_x))));
        } else{
            // Otherwise, it lies on the x-axis and the angle is zero
            orientation = 0;
        }
    } else{
        // If angle lies in 2nd quadrant, subtract calculated value from 180 degrees
        if(delta_y > 0){
            orientation = ((M_PI) - std::atan((std::fabs(delta_y)/std::fabs(delta_x))));
        } else if(delta_y < 0){
            // If angle lies in 3rd quadrant, add calculated value to 180 degrees
            orientation = ((M_PI) + std::atan((std::fabs(delta_y)/std::fabs(delta_x))));
        } else{
            // Otherwise, it lies on the x axis and the angle is 180 degrees
            orientation = M_PI;
        }
    }

    // Return the calculated angle
    return orientation;
}




void Controller::determineVelocityOmega(double & delta_orientation, double & l, double & delta_x){
    double velocity, omega;

    // If bogey located outside boundaries in either quadrant 1 or 2 and angle is positive, rotate counter clockwise at max ang speed
    if((delta_orientation >= (lower_limit_angle)) && (delta_orientation <= (M_PI))){
        velocity = sim_->V_TERM;
        omega = ((maximum_g_force_*gravity)/sim_->V_TERM);

    } else if((delta_orientation > (M_PI)) && (delta_orientation <= (upper_limit_angle))){
        // If bogey located outside boundaries in quadrant 3 or 4 and angle is positive, rotate clockwise at max ang speed
        velocity = sim_->V_TERM;
        omega =  (-1*((maximum_g_force_*gravity)/sim_->V_TERM));

    }
    else if ((delta_orientation <= (-1*lower_limit_angle)) && (delta_orientation >= (-1*M_PI))){
        // If bogey located outside boundaries in quadrant 3 or 4 and angle is negative, rotate clockwise at max ang speed
        velocity = sim_->V_TERM;
        omega = (-1 * ((maximum_g_force_*gravity)/sim_->V_TERM));

    } else if((delta_orientation < (-1*M_PI)) && (delta_orientation >= (-1*upper_limit_angle))){
        // If bogey located outside boundaries in quadrant 1 or 2 and angle is negative, rotate counter clockwise at max ang speed
        velocity = sim_->V_TERM;
        omega = ((maximum_g_force_*gravity)/sim_->V_TERM);

    }
    else{
        // If bogey is located within boundaries and in quadrant 4, rotate clockwise at speed proportional to max ang speed
        if((delta_orientation > 0) && (delta_orientation > lower_limit_angle)){
            omega = -1*(((2*delta_orientation)/(M_PI))*((maximum_g_force_*gravity)/sim_->V_TERM));

        } else if ((delta_orientation > 0) && (delta_orientation < lower_limit_angle)){
            // If bogey is within boundaries and in quadrant 1, rotate counter clockwise at speed proportional to max ang speed
            omega = (((2*delta_orientation)/(M_PI))*((maximum_g_force_*gravity)/sim_->V_TERM));

        } else if((delta_orientation < -1*upper_limit_angle)){
            // If bogey is within boundaries and in quadrant 1, rotate clockwise at a speed proportional to the max ang speed
            omega = -1*(((2*delta_orientation)/(M_PI))*((maximum_g_force_*gravity)/sim_->V_TERM));

        } else if((delta_orientation < 0) && (delta_orientation > -1*lower_limit_angle)){
            // If bogey is within boundaries and in quadrant 4, rotate counter clockwise at a speed proportional to the max ang speed
            omega = (((2*delta_orientation)/(M_PI))*((maximum_g_force_*gravity)/sim_->V_TERM));

        }

        // Calculate linear velocity using pure pursuit algorithm
        velocity = std::fabs((((std::pow(l,2))*omega)/(2*delta_x)));

        // Calculate g-force
        double g_force = ((velocity*omega)/gravity);

        // Check velocity is within operational limits
        if((velocity > sim_->MAX_V) || (velocity < sim_->V_TERM)){
            // If not, set to limit values
            if(velocity > sim_->MAX_V){
                velocity = sim_->MAX_V;
            } else{
                velocity = sim_->V_TERM;

            }

            // Re-calculate omega using pure pursuit algorithm
            if(omega > 0){
                omega = ((2*delta_x*velocity)/(std::pow(l,2)));

            } else{
                omega = (-1*((2*delta_x*velocity)/(std::pow(l,2))));

            }

            // Re-calculate g-force
            g_force = ((velocity*omega)/gravity);   
        }

        // If g-force is too high, re-calculate omega using g-force formula
        if(g_force > maximum_g_force_){
            omega = (((maximum_g_force_)*gravity)/velocity);

        } else if(g_force < (-1*maximum_g_force_)){
            omega = (-1*(((maximum_g_force_)*gravity)/velocity));

        }

        // If omega is low (less than one-quarter of the maximum rotational speed), set omega to zero and linear velocity to max speed
        if(std::fabs(omega) < (fraction_max_ang*((maximum_g_force_*gravity)/sim_->V_TERM))){
            omega = 0;
            velocity = sim_->MAX_V;
        }

    }

    // If control thread is already reading values, wait.
    while(reading_values_){

    }

    // Set member variables to appropriate values to control the aircraft.
    std::unique_lock<std::mutex> write_controls_lock(controls_mutex_);

    friendly_lin_velocity_ = velocity;
    friendly_ang_velocity_ = omega;

    write_controls_lock.unlock();
}




void Controller::calculateBogeyPosition(RangeBuffer & ranger_buffer){

        // Lock mutex and update necessary member variables needed in the calculation
        std::unique_lock<std::mutex> lock(friendly_mutex_);

        // Call getter for base station data
        base_data_ = ranger_buffer.getBaseData();

        // Call getter for friendly radar data
        friendly_data_ = ranger_buffer.getFriendlyData();

        // Get current friendly aircraft pose
        friendly_pose_ = sim_->getFriendlyPose();

        // Get most recent distance to the bogey from the base
        RangeStamped distance_base_bogey = base_data_.back();

        // Calculate average time interval between base data readings, used in calculations later.
        double average_time_interval = ((base_data_.back().timestamp - base_data_.front().timestamp)/(base_data_.size()));

        // Get last friendly data reading
        last_friendly_reading_ = friendly_data_.back();

        // Remove last friendly data reading
        friendly_data_.pop_back();

        // Get second last friendly data reading
        second_last_friendly_reading_ = friendly_data_.back();

        // Remove second last friendly data reading from deque
        friendly_data_.pop_back();

        // Get current time from simulator
        double current_time = sim_->elapsed();

        lock.unlock();

        Pose target_bogey_1;
        Pose target_bogey_2;

        // Solve simultaneous equations to obtain the two intersection points of the circles
        solveSimultaneousEquations(current_time, target_bogey_1, target_bogey_2, distance_base_bogey);

        // Set up temp vectors, used in calculations
        std::vector<Pose> temp_bogey_vector{target_bogey_1, target_bogey_2};

        std::vector<Pose> temp_target_pose(temp_bogey_vector.size());

        // If there has been multiple bogey positions calculated
        if(bogey_positions_.size() > 0){
            // Get previous bogey pose.
            std::vector<Pose> previous_bogey_pose = bogey_positions_.back();

            // Determine which target is more likely to be the bogey aircraft
            checkTargetToChase(temp_bogey_vector, previous_bogey_pose, average_time_interval);

            // For each bogey element calculate the bogey orientation and store in each bogey pose object

            double bogey_orientation;

            for (int i = 0; i < previous_bogey_pose.size(); i++){
                // Determine orientation of the bogey
                bogey_orientation = calculateOrientation(temp_bogey_vector.at(i), previous_bogey_pose.at(i));

                // Store orientation in temp vector
                temp_bogey_vector.at(i).orientation = bogey_orientation;

                // Calculate angular and linear speeds of bogey aircraft (rad/ms and m/ms respectively)
                double bogey_ang_speed = ((temp_bogey_vector.at(i).orientation - previous_bogey_pose.at(i).orientation)/(average_time_interval));
                double bogey_lin_speed = ((sim_->distance(temp_bogey_vector.at(i).position, previous_bogey_pose.at(i).position))/(average_time_interval));

                // Extrapolate target position orientation by using angular speed of bogey and average time interval
                double target_orientation = (temp_bogey_vector.at(i).orientation + (bogey_ang_speed*(1.5*(average_time_interval))));

                // Store in temp vector
                temp_target_pose.at(i).orientation = target_orientation;

                // Extrapolate target positions using linear speed of bogey and average time interval
                temp_target_pose.at(i).position.x = temp_bogey_vector.at(i).position.x + (bogey_lin_speed*(look_ahead_const*(average_time_interval)*(std::cos(target_orientation))));
                temp_target_pose.at(i).position.y = temp_bogey_vector.at(i).position.y + (bogey_lin_speed*(look_ahead_const*(average_time_interval)*(std::sin(target_orientation))));

            }
        }

        // Check size of bogey positions deque and remove oldest position if too large
        if(bogey_positions_.size() == bogey_position_size){
            bogey_positions_.pop_front();
        }

        // Store calculated positions of the bogey aircraft in deque
        bogey_positions_.push_back(temp_bogey_vector);

        // Check size of target positions deque and remove oldest position if too large
        if(target_positions_.size() == bogey_position_size){
            target_positions_.pop_front();
        }

        // Store latest calculated target position in deque
        target_positions_.push_back(temp_target_pose);


        std::unique_lock<std::mutex> second_lock(target_mutex_);
        // Update target point to chase
        target_ = target_positions_.back().at(currently_chasing_target_);

        // Notify all functions waiting on this condition variable
        cv_position_calculated_.notify_all();

        second_lock.unlock();

        // Set flag to true
        calculate_control_ = true;

        // Display predicted bogey position on simulation screen
        sim_->testPose(bogey_positions_.back().at(currently_chasing_target_));

        temp_target_pose.clear();

}




void Controller::calculateControlActions(){

    // Wait until the latest bogey positions are calculated
    std::unique_lock<std::mutex> lock(target_mutex_);

    while(!calculate_control_){
        cv_position_calculated_.wait(lock);
    }

    // Determine difference in x and y between current friendly position and target position.
    double delta_x = target_.position.x - friendly_pose_.position.x;
    double delta_y = target_.position.y - friendly_pose_.position.y;

    // Calculate angle between target position and friendly position
    double bogey_friendly_orientation = calculateOrientation(target_, friendly_pose_);

    lock.unlock();

    double delta_orientation;

    // Calculate look ahead distance used in pure pursuit control algorithm
    double l = (std::sqrt((std::pow(delta_x,2))+(std::pow(delta_y,2))));

    // Calculate angle to be rotated by friendly aircraft to point at bogey
    delta_orientation = bogey_friendly_orientation - friendly_pose_.orientation;

    // Calculate linear and angular velocity of friendly aircraft required to chase bogey
    determineVelocityOmega(delta_orientation, l, delta_x);

    calculate_control_ = false;
}




void Controller::controlFriendly(){

    // Lock mutex to protect member variables
    std::unique_lock<std::mutex> read_controls_lock(controls_mutex_);

    // Set flag so that mutex is not locked by calculation thread
    reading_values_ = true;

    // Control friendly aircraft with calculated linear and angular velocity values
    sim_->controlFriendly(friendly_lin_velocity_, friendly_ang_velocity_);

    read_controls_lock.unlock();

    reading_values_ = false;

}
