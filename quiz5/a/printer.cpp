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
  std::thread t1(printer, "The eagle has landed.");
  std::thread t2(printer, "The horse is in the barn.");
  std::thread t3(printer, "The chicken has flown the coop.");
  std::thread t4(printer, "The man in black fled across the desert.");

  t1.join();
  t2.join();
  t3.join();
  t4.join();

  return 0;
}
