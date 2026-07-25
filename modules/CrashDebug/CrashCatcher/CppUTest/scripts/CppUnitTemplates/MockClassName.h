#ifndef D_MockClassName_H
#define D_MockClassName_H

///////////////////////////////////////////////////////////////////////////////
//
//  MockClassName.h
//
//  MockClassName is responsible for providing a test stub for ClassName
//
///////////////////////////////////////////////////////////////////////////////
#include "ClassName.h"


class MockClassName : public ClassName
  {
  public:
    explicit MockClassName()
    {}
    virtual ~MockClassName()
    {}

  private:

    MockClassName(const MockClassName&);
    MockClassName& operator=(const MockClassName&);

  };

#endif  // D_MockClassName_H
