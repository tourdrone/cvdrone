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

struct directions{
  char forwards = 't';
  char backwards = 'g';
  char left = 'f';
  char right = 'h';
  char up = 'q';
  char down = 'a';
  char counterClockwise = 'r';
  char clockwise = 'y';
};
#endif
