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