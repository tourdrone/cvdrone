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

/*
  returns heading for control
*/
float detectObject(cv::Mat image, cv::KalmanFilter kalman, int minH, int maxH, int minS, int maxS, int minV, int maxV, bool learnMode, bool moveStatus, cv::Rect *rect) {

  int tolerance = 30;
  cv::Vec3b hsvSample;

  cv::Scalar green = CV_RGB(0,255,0); //putText color value
  
  // HSV image
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV_FULL);

  // Binalize
  cv::Mat binalized;
  cv::Scalar lower(minH, minS, minV);
  cv::Scalar upper(maxH, maxS, maxV);
  cv::inRange(hsv, lower, upper, binalized);

  // Show result
  cv::imshow("binalized", binalized);

  // De-noising
  cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(binalized, binalized, cv::MORPH_CLOSE, kernel);

  // Detect contours
  vector<vector<cv::Point> > contours;
  cv::findContours(binalized.clone(), contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

  // Find largest contour
  int contour_index = -1;
  double max_area = 0.0;
  for (size_t i = 0; i < contours.size(); i++) {
    double area = fabs(cv::contourArea(contours[i]));
    if (area > max_area) {
      contour_index = i;
      max_area = area;
    }
  }


  // Object detected
  if (contour_index >= 0) {
    // Moments
    cv::Moments moments = cv::moments(contours[contour_index], true);
    double marker_y = (int)(moments.m01 / moments.m00);
    double marker_x = (int)(moments.m10 / moments.m00);

    // Measurements
    cv::Mat measurement = (cv::Mat1f(2, 1) << marker_x, marker_y);

    // Correction
    cv::Mat estimated = kalman.correct(measurement);

    // Show result
    *rect = cv::boundingRect(contours[contour_index]);
    cv::rectangle(image, *rect, cv::Scalar(0, 255, 0));
  }

  // Prediction
  cv::Mat1f prediction = kalman.predict();
  int radius = 1e+3 * kalman.errorCovPre.at<float>(0, 0);

  // Show predicted position
  cv::circle(image, cv::Point(prediction(0, 0), prediction(0, 1)), radius, green, 2);
    
  // Calculate object heading fraction
  float heading = -((image.cols/2) - prediction(0, 0))/(image.cols/2);

  // Sample the object color
  if(learnMode) {
    // Show targeting crosshairs
    cv::line(image, cvPoint(image.cols/2, 0), cvPoint(image.cols/2, image.rows/2 - 2), green); //top vertical crosshair
    cv::line(image, cvPoint(image.cols/2, image.rows/2 + 2), cvPoint(image.cols/2, image.rows), green); //bottom vertical crosshair
    cv::line(image, cvPoint(0, image.rows/2), cvPoint(image.cols/2 - 2, image.rows/2), green); //left horizontal crosshair
    cv::line(image, cvPoint(image.cols/2 + 2, image.rows/2), cvPoint(image.cols, image.rows/2), green); //right horizontal crosshair

    hsvSample = hsv.at<cv::Vec3b>(cvPoint(image.cols/2, image.rows/2));

    setHSVTrackBarPositions(hsvSample[0], hsvSample[1], hsvSample[2], tolerance);
  }

  displayObjectFollowingInfo(&image, heading, hsvSample[0], hsvSample[1], hsvSample[2], moveStatus);

  return heading;
}

//Auto set Hue, Saturation, and Value tracking bars
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance) {
  //tolerance = 30;

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
