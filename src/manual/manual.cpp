/*
*/


#include "manual.h"

/*
*/
void ManualFlying::initializeManualFlying() {
  return;
}

/*
*/
void ManualFlying::closeManualFlying() {  
  return;
}

/*
*/
ControlMovements manualMovement(int key) {
  ControlMovements controlMovements;
  
  controlMovements.vx = 0;
  controlMovements.vy = 0;
  controlMovements.vz = 0;
  controlMovements.vr = 0;

  if (key == 't') { controlMovements.vx =  1.0; } //t key
  if (key == 'g') { controlMovements.vx = -1.0; } //g key
  if (key == 'f') { controlMovements.vy =  1.0; } //f key
  if (key == 'h') { controlMovements.vy = -1.0; } //h key
  if (key == 'q') { controlMovements.vz =  1.0; } //q key
  if (key == 'a') { controlMovements.vz = -1.0; } //a key
  if (key == 'r') { controlMovements.vr =  1.0; } //r key
  if (key == 'y') { controlMovements.vr = -1.0; } //y key

  return controlMovements;
}

/*
*/
void displayManualInfo(cv::Mat *image, ControlMovements controlMovements) {
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
