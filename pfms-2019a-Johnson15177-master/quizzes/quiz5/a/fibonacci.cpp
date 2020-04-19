#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <condition_variable>
#include <mutex>

using namespace std;

std::condition_variable cv;
std::mutex mtx;
bool dataready = false;

struct TDataBuffer {
  std::queue<long> buffer;
} sequence;

void fibonacci(void) {

  static long a = 0;
  static long b = 1;

  sequence.buffer.push(a);
  while(true) {
      unique_lock<mutex> lck(mtx);
    sequence.buffer.push(b);
    b = a + b;
    a = b - a;
    dataready = true;
    //cout << "im in"<< endl;
    cv.notify_all();
    lck.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

}

void printToTerminal(void) {

  while(true) {
      unique_lock<mutex> lck(mtx);
      while(dataready == false) {
          cv.wait(lck);
      }
      dataready = false;
    if(sequence.buffer.empty()) {
      std::cout << "<buffer was empty>" << std::endl;
      //cv.wait(lck);
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
