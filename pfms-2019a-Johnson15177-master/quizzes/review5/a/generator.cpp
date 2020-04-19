#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <condition_variable>

std::mutex m1, m2;
std::vector<double> buffer;
std::condition_variable cv;
bool ready = false;

//Generates random number between 0 and 10
double randomNumber(void) {
  unsigned int seed = std::chrono::system_clock::now()
    .time_since_epoch()
    .count();

  std::default_random_engine generator(seed);
  std::uniform_real_distribution<> value_distribution(0,10.0);
  return value_distribution(generator);
}


void generate(void) {
  while(true) {
//    m2.lock();

    std::unique_lock<std::mutex> lock(m1);
    std::cout << "...Generating number..." << std::endl;
    buffer.push_back(randomNumber());
    ready = true;

    cv.notify_all();

//    m1.lock();
//    m2.unlock();
//    m1.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void average(void) {
  while(true) {
    std::unique_lock<std::mutex> lock(m2);
    while(!ready){
        cv.wait(lock);
    }
    ready = false;
    std::cout << "...Calculating average... : ";
    double sum  = 0;
    for (int i: buffer) {
      sum += i;
    }
    double average = sum / (double) buffer.size();
    std::cout << average << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}


int main (void) {
  //Create the threads
  std::thread producer(generate);
  std::thread consumer(average);

  producer.join();
  consumer.join();

  std::cout << "Finished" << std::endl;

  return 0;
}
