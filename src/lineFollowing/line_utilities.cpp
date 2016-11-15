//
// Created by Caleb on 11/15/16.
//

#include "lineFollowing.h"
#include "line_utilities.h"


cv::Vec2f flip_line(cv::Vec2f line) {
  line[0] -= deg2rad(180);
  line[1] *= -1;
  return line;
}

double deg2rad(double deg) {
  return deg * (CV_PI / 180.0);
}

double rad2deg(double rad) {
  return rad * (180.0 / CV_PI);
}