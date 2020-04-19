#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "simulator.h"
#include <mutex>
#include <condition_variable>
#include <vector>


class Autopilot
{
public:
    Autopilot(std::shared_ptr<Simulator>);

    /**
    This function takes range readings from base to bogie and friendly to bogie and uses the intersection of circles to determine two possible bogie locations.

    */
    void setBogiePose(void);

    /**
    This function compares the current range between an estimated bogie position and friendly position with the previous bogie position and friendly position. If the previous range happens to be larger than the current position then the fucntion will return the second estimated bogie pose. Otherwise the first estimated bogie pose is returned.

    @return Pose - The bogies's x, y position (metres) and orientation (radians).
    */
    Pose getBogiePose(void);

    /**
    This function determines the orientation the friendly needs to point towards in order to get to the estimated bogie position. The angle is found using inverse tan. Once the angle is found, the function then determines the difference in which it must turn based on four separate cases. This function also sets the linear and angular velocities to max based on if the friendly is turning or not.

    @param targetX the bogie's estimated x position.
    @param targetY the bogie's estimated y position.
    */
    void setPursue(double targetX, double targetY);

    /**
    This function gets the angular velocity to contol the friendly.

    @return double - Angular velocity to control friendly.
    */
    double getAngControl(void);

    /**
    This function gets the linear velocity to contol the friendly.

    @return double - Linear velocity to control friendly.
    */
    double getLinControl(void);


private:
    Pose bogie1_; //!< One of two possible bogie positions
    Pose bogie2_; //!< One of two possible bogie positions
    double angVel_; //!< Angular velocity
    double linVel_; //!< Linear velocity
    double previousRange_; //!< Previous friendly to bogie range
    double bogieOrientation_; //!<  Estimated bogie orientation
    std::shared_ptr<Simulator> sim_; //!< Shared pointer of type simulator
};

#endif // AUTOPILOT_H
