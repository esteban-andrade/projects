#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

struct TDataBuffer {
    std::queue<long> buffer;
    std::mutex mu;
    std::condition_variable cv;
} sequence;

void fibonacci(void) {
    static long a = 0;
    static long b = 1;
    sequence.buffer.push(a);
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::unique_lock<std::mutex> lock(sequence.mu);

        sequence.buffer.push(b);

        sequence.cv.notify_all();

        b = a + b;
        a = b - a;


    }
}

void printToTerminal(void) {
    while(true) {
        while(sequence.buffer.empty()){
            std::unique_lock<std::mutex> lock(sequence.mu);
            sequence.cv.wait(lock);
        }

        if(sequence.buffer.empty()) {
            std::cout << "<buffer was empty>" << std::endl;
        } else {

            long next = sequence.buffer.front();
            sequence.buffer.pop();

            std::cout << "Next in sequence: " << next << std::endl;
        }
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
