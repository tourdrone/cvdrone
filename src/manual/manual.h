#ifndef MANUAL_HEADER
#define MANUAL_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

class ManualFlying {
  public:
    void initialize();
    void close();
    ControlMovements fly(int key);
};

#endif
