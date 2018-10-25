#ifndef STDDEVBUFFER_H
#define STDDEVBUFFER_H

#include <queue>
#include <math.h>

class StddevBuffer {

private:
  int    m_n;
  int    m_max_size;
  double m_sum;
  double m_sumsq;
  std::queue<double> m_buffer;

public:
  StddevBuffer(int max_size);
  double getStddev(double x);
};

#endif
