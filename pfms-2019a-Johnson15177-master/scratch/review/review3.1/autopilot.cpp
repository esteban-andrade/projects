#include "autopilot.h"
#include "cmath"
#include "iostream"

#define MAX_ANG_VEL 1.1
#define PI 3.14

Autopilot::Autopilot(const std::shared_ptr<Simulator> sim)
{
    sim_ = sim;
}

/* This function takes range readings from base to bogie and friendly to bogie and uses the intersection
   of circles to determine two possible bogie locations.*/
void Autopilot::setBogiePose(void)
{
    Pose friendlyPose = sim_->getFriendlyPose();
    double baseLocX = sim_->BSTATION_LOC.x;
    double baseLocY = sim_->BSTATION_LOC.y;
    auto baseLoc = sim_->BSTATION_LOC;

    double friendlyToBase = sim_->distance(baseLoc, friendlyPose.position);
    double bogieToBase = sim_->rangeToBogieFromBase().range;
    double bogieToFriendly = sim_->rangeToBogieFromFriendly().range;

    double distA = ((pow(bogieToFriendly, 2) - pow(bogieToBase, 2) + pow(friendlyToBase, 2)) / (2 * friendlyToBase));

    double perpHeight = sqrt(pow(bogieToFriendly, 2) - pow(distA, 2));
    double dotLocX = (friendlyPose.position.x + (distA * (baseLocX - friendlyPose.position.x))) / friendlyToBase;
    double dotLocY = (friendlyPose.position.y + (distA * (baseLocY - friendlyPose.position.y))) / friendlyToBase;

    bogie1_.position.x = ((dotLocX + (perpHeight * (baseLocY - friendlyPose.position.y))) / friendlyToBase);
    bogie1_.position.y = ((dotLocY - (perpHeight * (baseLocX - friendlyPose.position.x))) / friendlyToBase);

    bogie2_.position.x = ((dotLocX - (perpHeight * (baseLocY - friendlyPose.position.y))) / friendlyToBase);
    bogie2_.position.y = ((dotLocY + (perpHeight * (baseLocX - friendlyPose.position.x))) / friendlyToBase);

    previousRange_ = sim_->rangeToBogieFromFriendly().range;

}

/* This function compares the current range between an estimated bogie position and friendly position
   with the previous bogie position and friendly position. If the previous range happens to be larger
   than the current position then the fucntion will return the second estimated bogie pose. Otherwise
   the first estimated bogie pose is returned.*/
Pose Autopilot::getBogiePose(void)
{
    Pose friendlyPose = sim_->getFriendlyPose();

    double currentRange = sim_->rangeToBogieFromFriendly().range;

    if (currentRange < previousRange_)
    {
        Pose bogiePose1 = bogie1_;

        double Fx = friendlyPose.position.x;
        double Fy = friendlyPose.position.y;
        double grad = abs((bogiePose1.position.y - Fy) / (bogiePose1.position.x - Fx));
        bogieOrientation_ = atan(grad);
        bogiePose1.orientation = bogieOrientation_;

        return bogiePose1;
    }
    else
    {
        Pose bogiePose2 = bogie2_;

        double Fx = friendlyPose.position.x;
        double Fy = friendlyPose.position.y;
        double grad = abs((bogiePose2.position.y - Fy) / (bogiePose2.position.x - Fx));
        bogieOrientation_ = atan(grad);
        bogiePose2.orientation = bogieOrientation_;

        return bogiePose2;
    }

}


/* This function determines the orientation the friendly needs to point towards in order to get to
   the estimated bogie position. The angle is found using inverse tan. Once the angle is found, the
   function then determines the difference in which it must turn based on four separate cases. This
   function also sets the linear and angular velocities to max based on if the friendly is turning or not.*/
void Autopilot::setPursue(double targetX, double targetY)
{
    Pose friendlyPose = sim_->getFriendlyPose();

    double Fx = friendlyPose.position.x;
    double Fy = friendlyPose.position.y;
    double Fo = friendlyPose.orientation;
    double grad = (targetY - Fy) / (targetX - Fx);
    double theta = abs(atan(grad));

    if (Fy - targetY >= 0)
    {
        if (Fx - targetX <= 0)
        {
            theta = (2 * PI) - theta;
        }
        if (Fx - targetX > 0)
        {
            theta = PI + theta;
        }
    }
    else
    {
        if (Fx - targetX > 0)
        {
            theta = PI - theta;
        }
    }


    if (Fo >= theta)
    {
        if ((Fo - theta) <= PI)
        {
            if (Fo - theta < PI / 8)
            {
                angVel_ = 0;
            }
            else
            {
                angVel_ = -MAX_ANG_VEL;
            }
        }
        else
        {
            if (Fo - theta < PI / 8)
            {
                angVel_ = 0;
            }
            else
            {
                angVel_ = MAX_ANG_VEL;
            }
        }
    }
    else
    {
        if ((theta - Fo) <= PI)
        {
            if (theta - Fo < PI / 8)
            {
                angVel_ = 0;
            }
            else
            {
                angVel_ = MAX_ANG_VEL;
            }
        }
        else
        {
            if (theta - Fo < PI / 8)
            {
                angVel_ = 0;
            }
            else
            {
                angVel_ = MAX_ANG_VEL;
            }
        }
    }

    if (angVel_ == MAX_ANG_VEL || angVel_ == -MAX_ANG_VEL)
    {
        linVel_ = 50;

    }
    else if (angVel_ == 0)
    {
        linVel_ = 1000;
    }
}


/* This function returns the angular velocity needed for the friendly to reach the bogie*/
double Autopilot::getAngControl()
{
    double angVelocity = angVel_;
    return angVelocity;
}

/* This function returns the linear velocity needed for the friendly to reach the bogie*/
double Autopilot::getLinControl()
{
    double linVelocity = linVel_;
    return linVelocity;
}
