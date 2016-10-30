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


ControlMovements lineFollowingControl();

void line_main();

void detect_lines(cv::Mat &original_frame, double scale_factor);


#endif