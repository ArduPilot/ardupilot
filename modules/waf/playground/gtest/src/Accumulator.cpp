#include <stdlib.h>
#include "Accumulator.h"

Accumulator::Accumulator():m_total(0)
{
}

void Accumulator::accumulate(const char * data)
{
  m_total += strtol(data, 0, 0);
}

int Accumulator::total() const
{
  return m_total;
}
