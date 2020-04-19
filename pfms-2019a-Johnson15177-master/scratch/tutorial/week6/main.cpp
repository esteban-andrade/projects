#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>


using namespace std;

void sumQueue(queue<int> &numbers, mutex &mtx, int total) {
    mtx.lock();
    while (!numbers.empty()) {
    //Use mutex
        total += numbers.front();
        numbers.pop();
    //unlock so the other class
        mtx.unlock();
        mtx.lock();
    }
    mtx.unlock();
}


int main ()
{
    /* part 1 */
    queue<int> numbers;
    for (int i =1; i< 500000; i++)
    {
        numbers.push(i);
        numbers.push(-i);
    } 
    
    // We will use this mutex to synchonise access to num
    mutex mtx;
    int sub_total1=0;
    int sub_total2=0;
    
    // Create the threads
    thread t1(sumQueue,ref(numbers),ref(mtx), ref(sub_total1));
    thread t2(sumQueue,ref(numbers),ref(mtx), ref(sub_total2));

    // Wait for the threads to finish (they wont)
    t1.join();
    t2.join();

    cout<< "Subtotals " << sub_total1 << " " << sub_total2 << endl;
    cout<< "Combined total = " << sub_total1 + sub_total2 << endl;
    return 0;
}



