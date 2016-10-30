/*

*/

#include "lineFollowing.h"

using namespace std;

void LineFollowing::initialize() {
  return;
}

void LineFollowing::close() {
  return;
}

ControlMovements LineFollowing::fly(cv::Mat *image) {
  ControlMovements velocities;

  velocities.vx = 0;
  velocities.vy = 0;
  velocities.vz = 0;
  velocities.vr = 0;

  return velocities;
}
