/*
*/


#include "manual.h"

/*
*/
void ManualFlying::initialize() {
  return;
}

/*
*/
void ManualFlying::close() {  
  return;
}

/*
*/
ControlMovements ManualFlying::fly(int key) {
  ControlMovements velocities;
  
  velocities.vx = 0;
  velocities.vy = 0;
  velocities.vz = 0;
  velocities.vr = 0;

  if (key == 't') { velocities.vx =  1.0; } //t key
  if (key == 'g') { velocities.vx = -1.0; } //g key
  if (key == 'f') { velocities.vy =  1.0; } //f key
  if (key == 'h') { velocities.vy = -1.0; } //h key
  if (key == 'q') { velocities.vz =  1.0; } //q key
  if (key == 'a') { velocities.vz = -1.0; } //a key
  if (key == 'r') { velocities.vr =  1.0; } //r key
  if (key == 'y') { velocities.vr = -1.0; } //y key

  return velocities;
}

/*
*/
void ManualFlying::displayManualInfo(cv::Mat *image, ControlMovements controlMovements) {
  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  char vxDisplay[80]; //print buffer for x (forward/reverse) velocity
  char vyDisplay[80]; //print buffer for y (sideways) velocity
  char vzDisplay[80]; //print buffer for z (up/down) velocity
  char vrDisplay[80]; //print buffer for r rotational velocity

  sprintf(vxDisplay, "vx = %3.2f", controlMovements.vx);
  sprintf(vyDisplay, "vy = %3.2f", controlMovements.vy);
  sprintf(vzDisplay, "vz = %3.2f", controlMovements.vz);
  sprintf(vrDisplay, "vr = %3.2f", controlMovements.vr);

  putText(*image, vxDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, vyDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, vzDisplay, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, vrDisplay, cvPoint(30,120), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
}
