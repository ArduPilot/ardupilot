#pragma once

#include "genpybind.h"

class GENPYBIND(visible) Example {
public:
  static constexpr int GENPYBIND(hidden) not_exposed = 10;

  /// \brief Do a complicated calculation.
  int calculate(int some_argument = 5) const;

  GENPYBIND(getter_for(something))
  int getSomething() const;

  GENPYBIND(setter_for(something))
  void setSomething(int value);

private:
  int _value = 0;
};
