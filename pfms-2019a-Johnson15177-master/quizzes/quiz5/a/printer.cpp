#include <thread>
#include <iostream>
#include <mutex>
#include <string>
#include <chrono>

void printer(std::string message) {
    for (char c: message) {
      std::cout << c;
    }

    std::cout << std::endl;
}


int main (void) {
    std::mutex mtx;

    mtx.lock();
    std::thread t1(printer, "The eagle has landed.");
    mtx.unlock();
    mtx.lock();
    std::thread t2(printer, "The horse is in the barn.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mtx.unlock();
    mtx.lock();
    std::thread t3(printer, "The chicken has flown the coop.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mtx.unlock();
    mtx.lock();
    std::thread t4(printer, "The man in black fled across the desert.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mtx.unlock();

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    return 0;
}
