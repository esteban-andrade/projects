#ifndef RangeBUFFER_H
#define RangeBUFFER_H

#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <deque>

#include "simulator.h"

const int base_data_size = 5;
const int friendly_data_size = 10;
const int default_base_values_added = false;
const int default_friendly_values_added = false;

//!
//! \brief The RangeBuffer class
//!
//! The RangeBuffer class is a thread-safe class used to call related functions and store data in related deques from the radars attached to the base station and friendly aircraft.
//!
//! This class implements all mutexes and condition variables within the member functions, meaning access to all data within the class is thread-safe.
//!
class RangeBuffer{
public:
    //!
    //! \brief RangeBuffer
    //!
    //! Constructor for the RangeBuffer class used to configure all default values of the member variables. The constructor receives the simulator shared pointer variable to allow access to all Simulator member functions from within the class.
    //!
    //! \param simulator_passed - a reference to the simulator shared pointer used to portray the friendly and the bogey. This allows access to all the simulator member functions from within the class.
    //!
    RangeBuffer(const std::shared_ptr<Simulator> &);

    //!
    //! \brief addBaseData
    //!
    //! This is quite a simple function which works to continuously add data to the base_data_ deque.
    //!
    //! By calling the rangeToBogieFromBase() function and storing the result in a temporary variable, the delay required to mimic the sample time of the radar sensor is catered for. Upon completing this, a check is completed to ensure the base_data_ deque does not exceed the size limit of the deque. If it will, the first data point (the oldest piece of data from the radar) is popped from the front and the newest data is pushed to the back of the deque.
    //!
    //! This function then sends the condition variable notification and sets the base_values_added_ flag to true to allow any other function waiting on this data to continue its operation.
    //!
    void addBaseData();

    //!
    //! \brief addFriendlyData
    //!
    //! This function behaves very similar to the addBaseData function with the exception of calling the rangeToBogieFromFriendly() member function within the simulation.
    //!
    //! \sa addBaseData
    //!
    void addFriendlyData();

    //!
    //! \brief getBaseData
    //!
    //! A simple getter function used to return the base_data_ deque to the controller class to allow for calculations and control of the friendly aircraft.
    //!
    //! This getter waits until it receives notification from the cv_base_ condition variable, indicating that new data has been added to the deque before returning the data to the controller class.
    //!
    //! \return base_data_ - a deque of RangeStamped data entries, of the past 5 radar readings from the base station.
    //!
    std::deque<RangeStamped> getBaseData();

    //!
    //! \brief getFriendlyData
    //!
    //! Again, a simple getter used to return the friendly_data_ deque to the controller class to allow for calculations and control of the friendly aircraft.
    //!
    //! Similar to getBaseData(), this function waits to receive a notification from the cv_friendly_ condition variable, indicating that new data has been added to the deque before returning the data to the controller class.
    //!
    //! \return friendly_data_ - a deque of RangeStampe data entries of the past 10 radar readings from the friendly aircraft.
    //!
    std::deque<RangeStamped> getFriendlyData();

protected:



private:

    const std::shared_ptr<Simulator> sim_;//!< Simulation object used to call all method functions specific to the simulation

    std::deque<RangeStamped> base_data_;//!< deque used to store the radar data from the base station
    std::deque<RangeStamped> friendly_data_;//!< deque used to store the radar data from the friendly aircraft

    std::mutex friendly_mutex_;//!< Mutex used to lock access to the friendly_data_ variable to prevent data races
    std::mutex base_mutex_;//!< Mutex used to lock access to the base_data_ variable to prevent data races

    std::condition_variable cv_friendly_;//!< Condition variable used to notify when new data has been added to the friendly_data_ deque.
    std::condition_variable cv_base_;//!< Condition variable used to notify when new data has been added to the base_data_ deque.

    bool base_values_added_;//!< Boolean flag used to indicate when values have been added to the base_data_ deque.
    bool friendly_values_added_;//!< Boolean flag used to indicate when values have been added to the friendly_data_ deque.

};


#endif // RangeBUFFER_H
