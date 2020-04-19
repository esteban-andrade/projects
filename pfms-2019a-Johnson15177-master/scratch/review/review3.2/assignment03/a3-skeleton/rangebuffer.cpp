#include "rangebuffer.h"

#include <mutex>
#include <cmath>
#include <condition_variable>
#include <deque>


RangeBuffer::RangeBuffer(const std::shared_ptr<Simulator> & simulator_passed) :
    sim_(simulator_passed), base_values_added_(default_base_values_added), friendly_values_added_(default_friendly_values_added)
{

}


void RangeBuffer::addBaseData(){
    // Generate data from base station radar
    RangeStamped data = sim_->rangeToBogieFromBase();

    // Lock mutex to protect member variable
    std::unique_lock<std::mutex> lock(base_mutex_);

    // If the size of the deque has already reached the maximum size permitted, remove the oldest radar reading
    if(base_data_.size() == base_data_size){
        base_data_.pop_front();
    }

    // Add new data to back of deque
    base_data_.push_back(data);

    // Set flag as true and notify all functions waiting on the condition variable.
    base_values_added_ = true;

    cv_base_.notify_all();

    lock.unlock();
}

void RangeBuffer::addFriendlyData(){

    // Generate data from friendly aircraft radar
    RangeStamped data = sim_->rangeToBogieFromFriendly();

    // Lock mutex to protect member variable
    std::unique_lock<std::mutex> lock(friendly_mutex_);

    // If the size of the deque has already reached the maximum size permitted, remove the oldest radar reading
    if(friendly_data_.size() == friendly_data_size){
        friendly_data_.pop_front();
    }

    // Add new data to back of deque
    friendly_data_.push_back(data);

    // Set flag as true and notify all funcitons waiting on the condition variable
    friendly_values_added_ = true;

    cv_friendly_.notify_all();

    lock.unlock();

}

std::deque<RangeStamped> RangeBuffer::getBaseData(){

    // Wait for the condition variable to be notified
    std::unique_lock<std::mutex> lock(base_mutex_);

    while(!base_values_added_){
        cv_base_.wait(lock);
    }

    // Store return data in a temp variable
    std::deque<RangeStamped> value = base_data_;

    base_values_added_ = false;

    // Unlock mutex
    lock.unlock();

    // Return data
    return value;

}

std::deque<RangeStamped> RangeBuffer::getFriendlyData(){

    // Wait for the condition variable to be notified
    std::unique_lock<std::mutex> lock(friendly_mutex_);

    while(!friendly_values_added_){
        cv_friendly_.wait(lock);
    }

    // Store return data in a temp variable
    std::deque<RangeStamped> value = friendly_data_;

    friendly_values_added_ = false;

    // Unlock mutex
    lock.unlock();

    // Return data
    return value;

}





