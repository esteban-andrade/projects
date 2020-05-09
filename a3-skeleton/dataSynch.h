#ifndef DATASYNCH_H
#define DATASYNCH_H
#include <vector>
#include <deque>
#include <condition_variable>
#include "simulator.h"


class DataSynch{
    public:
        /*!
        Creates the datasynch object
        */
        DataSynch();
        /*!
        Obtains the Base Range Data
        @param[in] RangeStamped baseToBogie
        */
        void getBaseData(RangeStamped);
        /*!
        Checks the quantity of the container till it reaches the buffer size
        */
        void checkBaseSize();
        /*!
        Obtains the friendly range data
        @param[in] RangeStamped friendlyToBogie
        */
        void getFriendlyData(RangeStamped);
        /*!
        Checks the quantity of the container of the friendly to the BUFFER and interpolates the data as well
        @param[in] std::unique_lock<std::mutex>& locker
        */
        void checkFriendSize(std::unique_lock<std::mutex>& locker);
        /*!
        Displays the data to the terminal
        @param[in] std::unique_lock<std::mutex>& locker
        */
        void showData(std::unique_lock<std::mutex>& locker);
        /*!
        \return baseCompare
        */
        std::vector<long> getBaseCompare();
        /*!
        \return friendCompare
        */
        std::vector<long> getFriendCompare();
        /*!
        resets the containers
        */
        void resetData();
        /*!
        Interpolates the given data
        */
        double interpolate(long , long , double , double , double , bool );

    private:
        std::vector<double> timeFriend;
        std::vector<double> rangeFriend;
        std::deque<RangeStamped> baseContainer;
        std::deque<RangeStamped> friendlyContainer;
        std::condition_variable cv;
        std::condition_variable cv1;
        std::condition_variable cv2;
        std::vector<long> baseCompare;
        std::vector<long> friendCompare;

};

#endif