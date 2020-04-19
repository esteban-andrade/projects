#ifndef SAFEVECTOR_H
#define SAFEVECTOR_H

#include <mutex>
#include <chrono>
#include <condition_variable>
#include <string>
#include <vector>

class SafeVector {

public:
    SafeVector(std::string name);
    void addNumber(double number);
    void pruneValues(double min, double max);
    void pruneLength(unsigned max_size);

private:
    void printElements();

    std::string name_;
    std::vector<double> elements_;

    // Synchronisation
    std::mutex mutex_;
    std::condition_variable cv_;
    bool values_are_checked_, size_is_checked_;
};
#endif // SAFEVECTOR_H
