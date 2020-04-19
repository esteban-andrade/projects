#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include <random>

std::mutex m1, m2;
std::vector<double> buffer;

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
    m2.lock();

    std::cout << "...Generating number..." << std::endl;
    buffer.push_back(randomNumber());

    //m1.lock();
    m2.unlock();
    //m1.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void average(void) {
  while(true)
  {
    m1.lock();

    std::cout << "...Calculating average... : ";
    double sum  = 0;
    for (int i: buffer) {
      sum += i;
    }
    double average = sum / (double) buffer.size();
    std::cout << average << std::endl;

    //m2.lock();
    m1.unlock();
    //m2.unlock();

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
