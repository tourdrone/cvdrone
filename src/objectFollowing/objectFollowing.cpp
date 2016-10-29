/*
  Min Kao Drone Tour
  Modified by Elliot Greenlee
*/

#include "objectFollowing.h"
#include <string>
using namespace std;

const string THRESHOLDS_FILE_NAME = "thresholds.xml";

/*
*/
void initializeObjectFollowing(int *maxHue, int *minHue, int *maxSaturation, int *minSaturation, int *maxValue, int *minValue) {
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
void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value) {
  char headingDisplay[80]; //print buffer for heading
  char hsvSampleDisplay[80]; //print buffer for learning HSV values

  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  sprintf(headingDisplay, "heading = %+3.2f", heading); 
  sprintf(hsvSampleDisplay, "hsvSample = %3d, %3d, %3d", hue, saturation, value);

  putText(*image, headingDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, hsvSampleDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

}
