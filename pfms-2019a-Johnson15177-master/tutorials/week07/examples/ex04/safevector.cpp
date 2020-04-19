#include "safevector.h"

#include <iostream>

SafeVector::SafeVector(std::string name):
    name_(name), values_are_checked_(false), size_is_checked_(false)
{
}

void SafeVector::addNumber(double number) {
    std::unique_lock<std::mutex> lock(mutex_);
    elements_.push_back(number);
    values_are_checked_ = false;
    size_is_checked_ = false;
    cv_.notify_all();
    std::cout << "Added " << number << " to " << name_ << std::endl;
    printElements();
}

void SafeVector::pruneValues(double min, double max) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (values_are_checked_) {
        cv_.wait(lock);
    }
    std::cout << "Checking all values of " << name_ << std::endl;
    std::vector<double>::iterator it = elements_.begin();
    while (it != elements_.end()) {
        if (*it<min || *it>max) {
            std::cout << "Removing " << *it << " from " << name_ << std::endl;
            elements_.erase(it);
        } else {
            it++;
        }
    }
    values_are_checked_ = true;
    cv_.notify_all();
    std::cout << "Checked all values of " << name_ << std::endl;
    printElements();
}

void SafeVector::pruneLength(unsigned max_size) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (size_is_checked_ || !values_are_checked_) {
        cv_.wait(lock);
    }
    std::cout << "Checking length of " << name_ << std::endl;
    if (elements_.size() > max_size) {
        elements_.erase(elements_.begin());
        std::cout << "Pruned length of " << name_ << std::endl;
    } else {
        std::cout << "Length of " << name_ <<  " is ok" << std::endl;
    }
    size_is_checked_ = true;
    printElements();
}

void SafeVector::printElements()
{
    for (auto element : elements_) {
        std::cout << element << " ";
    }
    std::cout << std::endl << std::endl;
}
