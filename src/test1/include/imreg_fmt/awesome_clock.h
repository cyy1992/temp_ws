#ifndef AWESOME_CLOCK_H_
#define AWESOME_CLOCK_H_

#include <chrono>

class AwesomeClock
{
public:
  AwesomeClock() {}
  ~AwesomeClock() {}
  void start() { t_point_ = std::chrono::steady_clock::now(); }
  // return time elapsed(seconds) from start() called.
  double stop()
  {
    time_elapse_ = std::chrono::steady_clock::now() - t_point_;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time_elapse_)
               .count() *
           1.0e-9;
  }

private:
  std::chrono::steady_clock::time_point t_point_;
  std::chrono::duration<double> time_elapse_;
};

#endif
