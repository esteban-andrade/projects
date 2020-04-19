#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>

struct TDataBuffer {
  std::queue<long> buffer;
} sequence;

void fibonacci(void) {
  static long a = 0;
  static long b = 1;

  sequence.buffer.push(a);
  while(true) {
    sequence.buffer.push(b);
    b = a + b;
    a = b - a;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void printToTerminal(void) {
  while(true) {
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
