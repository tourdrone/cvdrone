#ifndef MANUAL_HEADER
#define MANUAL_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

class ManualFlying {
  public:
    void initializeManualFlying();
    void closeManualFlying();
};

/*
*/
ControlMovements manualMovement(int key);

/*
*/
void displayManualInfo(cv::Mat *image, ControlMovements controlMovements);


#endif
