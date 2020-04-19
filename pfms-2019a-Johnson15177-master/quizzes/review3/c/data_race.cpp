#include <iostream>
#include <thread>
#include <vector>

int shared_int (0);

void increment () {
   for (int i=0; i<10000000; ++i) {
      shared_int++;
   }
}

int main ()
{
    std::thread th1(increment);
    th1.join();
    std::thread th2(increment);
    th2.join();
    std::cout << "Final value: " << shared_int << std::endl;
    return 0;
}
