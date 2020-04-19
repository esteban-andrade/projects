#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <condition_variable>

using namespace std;

// The function generates samples
void generateSamples(vector<double> &data, mutex &mtx, condition_variable &cv)
{
    // Setup and seed our random number generator
    std::default_random_engine generator(
            std::chrono::system_clock::now().time_since_epoch().count());
    // Set mean and std dev of our distribution
    std::normal_distribution<double> distribution(6.0, 5.0);

    while (true) {
      // This delay is included to emulate the data rate of a sensor
        std::this_thread::sleep_for (std::chrono::milliseconds(500));

        // We can only obtain a lock in this thread if the mutex is not locked
        // anywhere else
        unique_lock<mutex> lck(mtx);
        cout << "Generating sample" << endl;

        // We only access `data` while the mutex is locked
        double sample = distribution(generator);
        data.push_back(sample);

        // Notfiy all threads waiting on this condition variable that something
        // has changed
        cv.notify_all();
    }
}

// This function consumes the samples
void processSamples(vector<double> &data, mutex &mtx, condition_variable &cv) {
    while (true) {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        unique_lock<mutex> lck(mtx);
        while (data.empty())
        {
            cv.wait(lck);
        }
        double sample = data.back();
        data.pop_back();
        cout <<  "sample is:" << sample << endl;
    }
}

int main ()
{
    vector<double> data;
    // We will use this mutex to synchonise access to num
    mutex mtx;
    condition_variable cv;

    // Create the threads
    thread inc_thread(generateSamples ,ref(data),ref(mtx), ref(cv));
    thread print_thread(processSamples,ref(data),ref(mtx), ref(cv));

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



