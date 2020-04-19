#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

using namespace std;

condition_variable cond;

//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer {
    mutex mu;
    std::queue<long> buffer;

} sequence;

void fibonacci(void) {
    unique_lock<mutex> locker(sequence.mu);
    static long a = 0;
    static long b = 1;
    sequence.buffer.push(a);
    locker.unlock();
    while(true) {
        locker.lock();
        sequence.buffer.push(b);
        b = a + b;
        a = b - a;
        locker.unlock();
        cond.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
    }
}

void printToTerminal(void) {

    while(true) {
        unique_lock<mutex> locker(sequence.mu);
        cond.wait(locker);
        if(sequence.buffer.empty()) {
            std::cout << "<buffer was empty>" << std::endl;
            locker.unlock();
        } else {
            long next = sequence.buffer.front();
            sequence.buffer.pop();
            locker.unlock();

            std::cout << "Next in sequence: " << next << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}


int main (void) {
    //Create the threads
    std::thread producer(fibonacci);
    std::thread consumer(printToTerminal);

    producer.join();
    consumer.join();
    return 0;
}
