/*

*/
#ifndef LINE_FOLLOWING_HEADER
#define LINE_FOLLOWING_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"
#include "pid.h"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

class Control;


class LineFollowing {
public:
  LineFollowing();

  LineFollowing(Control *control);

  void close();

  void fly();

  void detect_lines(cv::Mat &original_frame);

private:
  Control *control_ptr;

  double minH, minS, minV, maxH, maxS, maxV;
  double width, height;
  double kp, ki, kd;
  int time;
  ofstream myfile;
  PID *vertical_pid, *horizontal_pid;
  std::vector<cv::Vec2f> found_lines;
  int pause;

  double distance_from_center(float rho, float theta, double width, double height);

  cv::Point find_intersection(cv::Vec2f a, cv::Vec2f b);
};


enum direction {
  left, right, indeterminate
};

class line_options {


public:
  Vec2f horizontal, sloped;
  Vec2f vertical;
  direction d;

  line_options() {
    horizontal = Vec2f();
    vertical = Vec2f();
    sloped = Vec2f();
    d = indeterminate;
  }
};

#endif
