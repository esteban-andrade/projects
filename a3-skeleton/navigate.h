#include <cmath>
#include <deque>
#include "simulator.h"


class Navigate{
    public:
        /*!
        Creates the Navigate Object
        */
        Navigate();
        /*!
        Obtains the Orientation of the bogie relative to the friendly
        @param[in] std::deque<Pose> correctBogie, Pose friendly
        */
        void getVectorOrientation(std::deque<Pose> , Pose );
        /*!
        Moves the Friendly to one of the bogies position
        */
        void move();
        /*!
        Checks the friendly doesnt go out of the airspace
        */
        void checkAirspace(const std::shared_ptr<Simulator> & , Pose );
        /*!
        \return linSpeed
        */
        double getLinSpeed();
        /*!
        \return angVel
        */
        double getAngVel();

    private:
        double vectorOrientation;
        double offset;
        double linSpeed;
        double angVel;
};