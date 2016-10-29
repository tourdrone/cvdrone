/*
  Min Kao Drone Tour
  Modified by Elliot Greenlee
*/

#include "objectFollowing.h"
#include <string>
using namespace std;

const string THRESHOLDS_FILE_NAME = "thresholds.xml";
const double dt = 1.0; //Sampling time [s]

/*
*/
void initializeObjectFollowing(cv::KalmanFilter *kalman, int *maxHue, int *minHue, int *maxSaturation, int *minSaturation, int *maxValue, int *minValue) {
  // XML save data for object following color thresholds
  cv::FileStorage fs(THRESHOLDS_FILE_NAME, cv::FileStorage::READ);

  // If there is a save file then read it
  if (fs.isOpened()) {
    *maxHue = fs["H_MAX"];
    *minHue = fs["H_MIN"];
    *maxSaturation = fs["S_MAX"];
    *minSaturation = fs["S_MIN"];
    *maxValue = fs["V_MAX"];
    *minValue = fs["V_MIN"];
    fs.release();
  }

  // Create a window
  cv::namedWindow("binalized");
  cv::createTrackbar("Hue max", "binalized", maxHue, 255);
  cv::createTrackbar("Hue min", "binalized", minHue, 255);
  cv::createTrackbar("Saturation max", "binalized", maxSaturation, 255);
  cv::createTrackbar("Saturation min", "binalized", minSaturation, 255);
  cv::createTrackbar("Value max", "binalized", maxValue, 255);
  cv::createTrackbar("Value min", "binalized", minValue, 255);
  cv::resizeWindow("binalized", 0, 0);

  // Transition matrix (x, y, vx, vy)
  cv::Mat1f A(4, 4);
  A << 1.0, 0.0,  dt, 0.0,
       0.0, 1.0, 0.0,  dt,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0;
  kalman->transitionMatrix = A;

  // Measurement matrix (x, y)
  cv::Mat1f H(2, 4);
  H << 1, 0, 0, 0,
       0, 1, 0, 0;
  kalman->measurementMatrix = H;

  // Process noise covairance (x, y, vx, vy)
  cv::Mat1f Q(4, 4);
  Q << 1e-5,  0.0,  0.0,  0.0,
        0.0, 1e-5,  0.0,  0.0,
        0.0,  0.0, 1e-5,  0.0,
        0.0,  0.0,  0.0, 1e-5;
  kalman->processNoiseCov = Q;

  // Measurement noise covariance (x, y)
  cv::Mat1f R(2, 2);
  R << 1e-1,  0.0,
        0.0, 1e-1;
  kalman->measurementNoiseCov = R;
}

/*
*/
void closeObjectFollowing(int maxHue, int minHue, int maxSaturation, int minSaturation, int maxValue, int minValue) {
  //Save thresholds
  cv::FileStorage fs(THRESHOLDS_FILE_NAME, cv::FileStorage::READ);
  fs.open(THRESHOLDS_FILE_NAME, cv::FileStorage::WRITE);
  if (fs.isOpened()) {
    cv::write(fs, "H_MAX", maxHue);
    cv::write(fs, "H_MIN", minHue);
    cv::write(fs, "S_MAX", maxSaturation);
    cv::write(fs, "S_MIN", minSaturation);
    cv::write(fs, "V_MAX", maxValue);
    cv::write(fs, "V_MIN", minValue);
    fs.release();
  }
}

//Auto set Hue, Saturation, and Value tracking bars
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance) {
  tolerance = 30;

  cv::setTrackbarPos("Hue max", "binalized", hue + tolerance);
  cv::setTrackbarPos("Hue min", "binalized", hue - tolerance);

  cv::setTrackbarPos("Saturation max", "binalized", saturation + tolerance);
  cv::setTrackbarPos("Saturation min", "binalized", saturation - tolerance);
      
  cv::setTrackbarPos("Value max", "binalized", value + tolerance);
  cv::setTrackbarPos("Value min", "binalized", value - tolerance);

}

/*
*/
void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value, bool moveStatus) {
  char headingDisplay[80]; //print buffer for heading
  char hsvSampleDisplay[80]; //print buffer for learning HSV values
  char moveStatusDisplay[80]; //print buffer for stop/go status

  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  sprintf(headingDisplay, "heading = %+3.2f", heading); 
  sprintf(hsvSampleDisplay, "hsvSample = %3d, %3d, %3d", hue, saturation, value);
  if (moveStatus) {
    sprintf(moveStatusDisplay, "move status = GO");
  }
  else {
    sprintf(moveStatusDisplay, "move status = STOP");
  }

  putText(*image, headingDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, hsvSampleDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, moveStatusDisplay, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

}
