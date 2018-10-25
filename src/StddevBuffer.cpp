#include "StddevBuffer.h"

/* StddevBuffer() //{ */

StddevBuffer::StddevBuffer(int max_size) {

  m_n = 0;
  m_sum      = 0.0;
  m_sumsq    = 0.0;
  m_max_size = max_size;
}

//}

/* getStddev() //{ */

double StddevBuffer::getStddev(double x) {

  m_sum += x;
  m_sumsq += std::pow(x, 2);
  m_n++;

  m_buffer.push(x);
  if (m_n > m_max_size) {
    m_sum -= m_buffer.front();
    m_sumsq -= std::pow(m_buffer.front(), 2);
    m_buffer.pop();
    m_n--;
  }

  if (m_n > 2) {
    double var = 1.0 / (m_n - 1.0) * (m_sumsq - std::pow(m_sum, 2) / m_n);
    return std::sqrt(var);
  } else {
    return 1.0;
  }
}

//}
