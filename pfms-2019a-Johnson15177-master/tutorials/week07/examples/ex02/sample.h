#ifndef SAMPLE_H
#define SAMPLE_H

#include <condition_variable>
#include <mutex>
#include <vector>


class Sample
{
public:
  Sample();

  void addSample(double value);
  double getSample();
private:
    std::vector<double> data; //! storage for data
    std::mutex mtx; //! mutex for locking
    std::condition_variable cv; //! convar

};

#endif // SAMPLE_H
