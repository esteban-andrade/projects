#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "simulator.h"
#include "rangebuffer.h"

#include <deque>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>


const double gravity = 9.81;
const int bogey_position_size = 5;

const double default_ang_velocity = 0.0;
const double default_magnitude_of_max = 0.999;
const bool default_out_of_airspace = false;
const int default_target = 0;
const bool default_reading_values = false;
const bool default_calculate_control = false;

const double check_target_range_boundary = 300.0;
const double lower_limit_angle = 0.2*M_PI;
const double upper_limit_angle = 1.6*M_PI;
const double fraction_max_ang = 0.25;
const double look_ahead_const = 1.5;
const double MF_friendly_radar = 1.1;
const double MF_distance_between_targets = 1.5;


//!
//! \brief The Controller class
//!
//! The Controller class is used to provide all control functionality to the friendly plane.
//!
//! This class calculates the estimated position(s) of the bogey plane, performs calculations of necessary control actions to reach the bogey plane and controls the friendly plane accordingly.
//!
class Controller{
public:
    //!
    //! \brief Controller
    //! Constructor of Controller object used to initialise all member variables to default values.
    //!
    //! This constructor stores the simulator as a member variable to allow access to the member functions of the simulator object.
    //!
    //! \param simulator_passed - reference to the simulator shared pointer. This is saved into a member variable to allow direct access to the simulator from within the class.
    //!
    Controller(const std::shared_ptr<Simulator> &);


    //!
    //! \brief calculateBogeyPosition
    //!
    //! This function is used to estimate the two locations of the bogey. By knowing the position of the friendly plane and base station, the distance from the base station to the bogey and the position from the friendly to the bogey; two equations of circles can be determined: one with the centre as the base station and one as the centre of the friendly. These equations are then solved simultaneously to determine the two intersection points of the circles (two possible locations of the bogey).
    //!
    //! For each of the calculated positions of the bogey, they are compared against the previously calculated positions to determine the change in position, and consequently the orientation of the bogey. Using this information and the average time taken between each measurement from the base (calculated by subtracting the latest timestamp from the first timestamp and dividing by the size of the base_data_ deque), the linear and angular velocities of the bogey plane are calculated.
    //!
    //! Using this information, the target location to control the friendly to aim for (just ahead of the bogey) can be extrapolated. By knowing the linear and angular velocities of the bogey plane, using simple mathematics and the average time between readings calculated as described above, the next location of the bogey can be extrapolated. By then multiplying this by a magnification factor, the friendly plane can always be aimed just ahead of the bogey, allowing it to be intercepted quickly.
    //!
    //! The locations of the bogey are stored in the bogey_positions_ deque and the target positions are stored in a deque of equivalent size. A condition variable is flagged upon completing the calculation to notify the calculation function to start
    //!
    //! \param ranger_buffer - The RangeBuffer object used to call the getters to obtain the base and friendly radar data.
    //!
    //! \sa checkTargetToChase(), solveSimultaneousEquations(), calculateOrientation()
    //!
    void calculateBogeyPosition(RangeBuffer & ranger_buffer);


    //!
    //! \brief calculateControlActions
    //!
    //! This function waits for the condition variable to be notified by the calculateBogeyPosition function before calculating the linear and angular velocity of the friendly plane required to track the bogey.
    //!
    //! This function determines the difference in orientation from the friendly orientation and the location of the bogey from the friendly (the angle that needs to be rotated by the bogey).
    //!
    //! This is then passed to the determineVelocityOmega function to calculate the required values for the friendly aircraft.
    //!
    //! \sa determineVelocityOmega()
    //!
    void calculateControlActions();


    //!
    //! \brief controlFriendly
    //!
    //! A simple function called by the Control thread in main.cpp to set the linear and angular velocity of the friendly.
    //!
    //! This function is separate to the calculation of control actions function to prevent any possibility of the watchdog timer not being fed whilst the control actions are calculated. If the new control action has not been calculated, the friendly plane will simply be set the values that were previously stored in the member variables (repeating the instruction).
    //!
    void controlFriendly();

protected:


private:
    //!
    //! \brief checkTargetToChase
    //!
    //! Upon calculating the two potential locations of the bogey, this function is utilised to check which of the two intersection points is more likely to be the bogey plane and to chase it accordingly.
    //!
    //! To prevent multiple 'triggers' of the logic checks in this function a bool value is flagged whenever the target is changed, allowing only a single logic check (with the exception of the checking nan position) to change the target to be chased with each iteration of the calculation thread.
    //!
    //! When tracking the enemy bogey, some unusual activity was noticed in which the target point it was chasing would unexpectedly switch between the two alternatives, hence, in an attempt to minimise the amount of switching and strive to track the enemy bogey consistently the following logic was implemented:
    //!
    //! 1. If the position of the enemy target returns nan, track the most recent, real value for this target. This logic was implemented to cater for the situation that the distance between the enemy bogey and the base station was less than the distance between the friendly aircraft and the bogey. In this situation, only one real intersect point would exist and the other would return nan. If a target pose returned nan, the previous real pose was stored as the target. In doing so, the friendly aircraft would always track a real target pose.
    //!
    //! 2. The second element of logic in this function is related to the position of the target points with respect to the base station. It is required the friendly aircraft remains in the airspace. Hence a check is done to see if the distance between each target point and the base station is greater than the radius of the airspace. If so, the friendly aircraft responds in one of two ways:
    //!     - Switch from chasing the current target point to the other possible enemy target point, OR
    //!     - If both target points are out of the airspace, the friendly aircraft is told to return to the base station and hover around it until the enemy bogey re-enters the airspace.
    //!
    //! 3. Thirdly, a comparison is made between the last two radar readings from the friendly radar. A check is done to see if the last reading from the friendly radar is greater than the second last reading multiplied by a magnification factor. If so, it can be concluded that the distance between the bogey and friendly is growing and consequently, it can be said that the friendly aircraft is driving away from the bogey (and hence, chasing the wrong target point). With this being the case, the friendly aircraft realises it is chasing the incorrect target point and switches to start chasing the other one.
    //!
    //! 4. Lastly, if the target point has not been changed up until this check, a comparison is completed between the last two positions of each bogey target position. In the event that the distance travelled between these two points is larger than physically possible by the plane, it can be concluded that the friendly aircraft has crossed the midpoint of the two circles, causing the calculated target points to switch. In the event that this has happened, the target point which the friendly aircraft is chasing is switched to ensure the friendly continues to chase the same physical point in space.
    //!
    //! \param temp_bogey_vector - A vector of Poses containing the most recently calculated possible locations of the bogey plane.
    //! \param previous_bogey_vector - A vector of Poses containing the previous calculated positions of the bogey plane, used to compare the current readings against.
    //! \param average_time_interval - average time between base radar readings as calculated earlier in the function.
    //!
    //!
    void checkTargetToChase(std::vector<Pose> &, std::vector<Pose> &, double &);


    //!
    //! \brief solveSimultaneousEquations
    //!
    //! This function is used to calculate the two intersection points of the circles obtained. Knowing the base station is located at position (0,0) and knowing the location of the friendly aircraft; two circle equations can be derived and solved simultaneously to determine the coordinates of the two intersection points of the circles. These two points then become the two possible locations of the bogey plane.
    //!
    //! Firstly, the timestamps of the current point in time (which is the time at which the last base radar reading was received) and the last friendly radar reading are compared. This is done to determine whether the distance from the friendly aircraft to the bogey should be interpolated or extrapolated from known values.
    //!
    //! After this, the required equations are developed and used to solve for the two intersection points of the circles. These values are stored in the target_bogey_1 and target_bogey_2 variables which are later utilised to determine which is the bogey plane and hence, which intersection point the friendly aircraft should chase.
    //!
    //! \param current_time - the current time interval (calculated by a call to the timer in the simulation object)
    //! \param target_bogey_1 - the storage location for the first intersection point passed as reference to allow multiple values to be 'returned' from the function
    //! \param target_bogey_2 - the storage location for the second intersection point passed as reference to allow multiple values to be 'returned' from the function
    //! \param distance_base_bogey - the last radar reading from the base station to the bogey aircraft, used in generating the circle equations
    //!
    void solveSimultaneousEquations(double &, Pose &, Pose &, RangeStamped &);


    //!
    //! \brief calculateOrientation
    //!
    //! This is a simple mathematical function utilised to calculate the angle between two points of interest. Being given the Pose of the two points, an angle is calculated using trigonometry and returned from the function.
    //!
    //! \param Pose1 - the pose of the first point between which the angle is to be calculated
    //! \param Pose2 - the pose of the second point between which the angle is to be calculated
    //!
    //! \return orientation - the calculated angle between the two points of interest
    //!
    double calculateOrientation(Pose &, Pose &);


    //!
    //! \brief determineVelocityOmega
    //!
    //! This function is utilised during the calculation of the control actions for the friendly aircraft to determine the values for linear and angular velocity of the friendly plane that will allow it to chase the bogey plane.
    //!
    //! The values for each of these control variables are determined largely on the respective position of the bogey aircraft from the friendly aircraft.
    //!
    //! An initial check is done of the size of the delta_orientation parameter. If this value exceeds the defined boundaries, the friendly aircraft is flown at the minimum linear velocity and the maximum possible angular velocity is calculated based on g-force constraints.
    //!
    //! If the value lies within the defined boundaries, using a proportional relationship between angle to be rotated and maximum angular velocity, the new angular velocity can be calculated. Using this new value for omega, the linear velocity is calculated using the pure pursuit formula.
    //!
    //! Following this, a number of checks of the linear velocity are completed:
    //!
    //! - If the linear velocity calculated exceeds the maximum velocity, it is set to the maximum linear velocity and omega is re-calculated using the pure pursuit formula.
    //! - If the linear velocity calculated is less than the terminal velocity, it is set to the terminal velocity and omega is re-calculated using the pure pursuit formula.
    //!
    //! After this the g-forces are checked to make sure they are compliant and if they exceed maximum allowable g-force, the angular velocity is re-calculated using the g-force formula. Angular velocity is chosen here over linear velocity to provide a bias towards the linear velocity. In this instance, the friendly aircraft will prioritise flying closer to the bogey aircraft over orientating to it. This allows the friendly aircraft to fly within a closer range, in an attempt to catch the bogey plane faster.
    //!
    //! Lastly, a check is completed if the angular velocity of the plane is very low, it is set to zero and the linear velocity is set to its maximum value (friendly plane will drive straight as fast as possible).
    //!
    //! A check is done, if the linear and angular velocity variables are currently being read by the control thread, this thread will wait until it is done. The calculated values for linear and angular velocity are then written to the appropriate member variables to control the friendly aircraft.
    //!
    //! \param delta_orientation - The difference in orientation between the orientation of the bogey plane from the friendly and the current orientation of the friendly plane (i.e. the angle that needs to be rotated to face the bogey plane)
    //! \param l - The look ahead distance between the friendly plane and the bogey plane utilised in the pure pursuit algorithm to chase the bogey.
    //! \param delta_x - The distance between the friendly and bogey aircraft in the x direction. Utilised in the pure pursuit algorithm to chase the bogey plane
    //!
    void determineVelocityOmega(double & , double &, double &);



    const std::shared_ptr<Simulator> sim_;//!< Simulation object used to call all method functions specific to the simulation

    std::atomic<double> friendly_lin_velocity_;//!< member variable used to store the linear velocity of the friendly aircraft to track the bogey
    std::atomic<double> friendly_ang_velocity_;//!< member variable used to store the angular velocity of the friendly aircraft to track the bogey

    int currently_chasing_target_;//!< Variable used to store which intersection point should be chased by the friendly bogey.

    std::deque<RangeStamped> base_data_;//!< deque used to store the radar data received from the base station
    std::deque<RangeStamped> friendly_data_; //!< deque used to store the radar data received from the friendly aircraft

    std::deque<std::vector<Pose>> bogey_positions_;//!< deque used to store the calculated bogey locations
    std::deque<std::vector<Pose>> target_positions_;//!< deque used to store the calculated target locations

    RangeStamped last_friendly_reading_;//!< RangeStamped variable used to store the last reading from the friendly aircraft, utilised to extrapolate distance between friendly and bogey in solveSimultaneousEquations function
    RangeStamped second_last_friendly_reading_;//!< RangeStamped variable used to store the second last reading from the friendly aircraft, utilised to extrapolate distance between friendly and bogey in solveSimultaneousEquations function

    Pose friendly_pose_;//!< Pose variable used to store the current pose of the friendly aircraft used throughout the calculations
    Pose target_;//!< Pose variable used to store the target pose to chase. This is the pose just ahead of the bogey intersection point currently being chased


    std::mutex friendly_mutex_;//!< Mutex used to lock member variables as they are updated from the range buffer class
    std::mutex target_mutex_;//!< Mutex used to lock the target member variable whilst it is updated
    std::mutex controls_mutex_;//!< Mutex used to lock the friendly linear and angular velocity member variables whilst they are being read/updated.

    std::condition_variable cv_position_calculated_;//!< Condition variable used to notify other functions that the latest bogey positions have been calculated and to re-calculate the control actions to suit it

    bool calculate_control_;//!< Flag set true when the position of the bogey plane has been calculated and to indicate that the control actions should be calculated to track the plane.
    bool out_of_airspace_;//!< Flag used to indicate that the bogey has flown out of the airspace and for the friendly aircraft to return to the base station
    bool reading_values_;//!< Flag used to indicate that the friendly linear and angular velocities are currently being read by the control function and to tell the calculation function to wait before updating the control variables.

    double maximum_g_force_; //!< member variable used to calculate the maximum g force as a function of the simulator max g-force variable. This is because we were told we were not allowed to equal the simulator max G, we must be less than it.

};


#endif // CONTROLLER_H
