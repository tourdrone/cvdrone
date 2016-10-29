
#include "manual.h"

void displayManualInfo(cv::Mat *image, double vx, double vy, double vz, double vr) {
  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  char vxDisplay[80]; //print buffer for x (forward/reverse) velocity
  char vyDisplay[80]; //print buffer for y (sideways) velocity
  char vzDisplay[80]; //print buffer for z (up/down) velocity
  char vrDisplay[80]; //print buffer for r rotational velocity

  sprintf(vxDisplay, "vx = %3.2f", vx);
  sprintf(vyDisplay, "vy = %3.2f", vy);
  sprintf(vzDisplay, "vz = %3.2f", vz);
  sprintf(vrDisplay, "vr = %3.2f", vr);

  putText(*image, vxDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, vyDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, vzDisplay, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, vrDisplay, cvPoint(30,120), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
}
