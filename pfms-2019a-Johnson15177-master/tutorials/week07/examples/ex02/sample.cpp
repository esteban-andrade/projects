#include "sample.h"

using namespace std;


Sample::Sample()
{

}

void Sample::addSample(double value){

    // We can only obtain a lock in this thread if the mutex is not locked
    // anywhere else
    unique_lock<mutex> lck(mtx);

    // Once locked we can get acces to data
    data.push_back(value);

    // Notfiy all threads waiting on this condition variable that something
    // has changed
    cv.notify_all();
}

double Sample::getSample(){

    // We can only obtain a lock in this thread if the mutex
    // is not locked anywhere else
    unique_lock<mutex> lck(mtx);

    // We wait until data is unlocked
    while (data.empty()) {
        cv.wait(lck);
    }

    double value = data.back();
    data.pop_back();

    return value;
}

