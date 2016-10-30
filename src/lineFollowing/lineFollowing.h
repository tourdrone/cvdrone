/*
*/

#ifndef LINE_FOLLOWING_HEADER
#define LINE_FOLLOWING_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

/*
*/
class LineFollowing {
  public:
    void initialize();
    void close();
    ControlMovements fly(cv::Mat *image);
};

#endif
