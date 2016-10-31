#ifndef MANUAL_HEADER
#define MANUAL_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

class Control;

class ManualFlying {
  public:
    ManualFlying(Control *control);
    void close();
    void fly();
  private:
    Control *control_ptr;
};

#endif
