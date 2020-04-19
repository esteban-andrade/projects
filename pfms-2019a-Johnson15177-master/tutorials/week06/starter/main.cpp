#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

using namespace std;

void incrementNum(int &num, mutex &numMutex) {
    while (true) {
    //Use mutex 
    }
}

void printNum(int &num, mutex &numMutex) {
    while (true) {
    //Use mutex 
    }
}

int main ()
{
    int num = 0;
    // We will use this mutex to synchonise access to num
    mutex numMutex;

    // Create the threads
    thread inc_thread(incrementNum,ref(num),ref(numMutex));
    thread print_thread(printNum,ref(num),ref(numMutex));

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



