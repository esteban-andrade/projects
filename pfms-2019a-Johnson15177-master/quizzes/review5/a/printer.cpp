#include <thread>
#include <iostream>
#include <mutex>
#include <string>
#include <chrono>

void printer(std::string message, std::mutex & mu) {



    mu.lock();
    for (char c: message) {
      std::cout << c;
    }

    std::cout << std::endl;
    mu.unlock();


}


int main (void) {
  std::mutex mu;

  std::thread t1(printer, "The eagle has landed.", std::ref(mu));
  std::thread t2(printer, "The horse is in the barn.", std::ref(mu));
  std::thread t3(printer, "The chicken has flown the coop.", std::ref(mu));
  std::thread t4(printer, "The man in black fled across the desert.", std::ref(mu));

  t1.join();
  t2.join();
  t3.join();
  t4.join();

  return 0;
}
