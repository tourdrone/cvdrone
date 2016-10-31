/*
  Min Kao Drone Tour
  Modified by Elliot Greenlee
*/

#include "objectFollowing.h"
#include "../control.h"
#include <string>
using namespace std;

const string THRESHOLDS_FILE_NAME = "thresholds.xml";
const double dt = 1.0; //Sampling time [s]

/*
*/
ObjectFollowing::ObjectFollowing(Control *control) {
  control_ptr = control;
  kalman = cv::KalmanFilter(4, 2, 0);

  // XML save data for object following color thresholds
  cv::FileStorage fs(THRESHOLDS_FILE_NAME, cv::FileStorage::READ);

  // If there is a save file then read it
  if (fs.isOpened()) {
    maxH = fs["H_MAX"];
    minH = fs["H_MIN"];
    maxS = fs["S_MAX"];
    minS = fs["S_MIN"];
    maxV = fs["V_MAX"];
    minV = fs["V_MIN"];
    fs.release();
  }

  // Create a window
  cv::namedWindow("binalized");
  cv::createTrackbar("Hue max", "binalized", &maxH, 255);
  cv::createTrackbar("Hue min", "binalized", &minH, 255);
  cv::createTrackbar("Saturation max", "binalized", &maxS, 255);
  cv::createTrackbar("Saturation min", "binalized", &minS, 255);
  cv::createTrackbar("Value max", "binalized", &maxV, 255);
  cv::createTrackbar("Value min", "binalized", &minV, 255);
  cv::resizeWindow("binalized", 0, 0);

  // Transition matrix (x, y, vx, vy)
  cv::Mat1f A(4, 4);
  A << 1.0, 0.0,  dt, 0.0,
       0.0, 1.0, 0.0,  dt,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0;
  kalman.transitionMatrix = A;

  // Measurement matrix (x, y)
  cv::Mat1f H(2, 4);
  H << 1, 0, 0, 0,
       0, 1, 0, 0;
  kalman.measurementMatrix = H;

  // Process noise covairance (x, y, vx, vy)
  cv::Mat1f Q(4, 4);
  Q << 1e-5,  0.0,  0.0,  0.0,
        0.0, 1e-5,  0.0,  0.0,
        0.0,  0.0, 1e-5,  0.0,
        0.0,  0.0,  0.0, 1e-5;
  kalman.processNoiseCov = Q;

  // Measurement noise covariance (x, y)
  cv::Mat1f R(2, 2);
  R << 1e-1,  0.0,
        0.0, 1e-1;
  kalman.measurementNoiseCov = R;
}

/*
*/
void ObjectFollowing::close() {
  //Save thresholds
  cv::FileStorage fs(THRESHOLDS_FILE_NAME, cv::FileStorage::READ);
  fs.open(THRESHOLDS_FILE_NAME, cv::FileStorage::WRITE);
  if (fs.isOpened()) {
    cv::write(fs, "H_MAX", maxH);
    cv::write(fs, "H_MIN", minH);
    cv::write(fs, "S_MAX", maxS);
    cv::write(fs, "S_MIN", minS);
    cv::write(fs, "V_MAX", maxV);
    cv::write(fs, "V_MIN", minV);
    fs.release();
  }
}

/*
  returns heading for control
*/
void ObjectFollowing::fly() {

  ControlMovements *controlMovements = &(control_ptr->velocities);
  cv::Mat *image = &(control_ptr->image);

  int tolerance = 50;
  cv::Vec3b hsvSample;
  cv::Scalar green = CV_RGB(0,255,0); //putText color value

    //switch between learning and non-learning mode
    if (control_ptr->key == 'l') {
      learnMode = !learnMode;
      if (learnMode) {
        printf("Learning mode is enabled\n");
        printf("The color at the crosshairs is being learned\n");
        printf("Press l again to turn off learning mode\n\n");
      }
      else {
        printf("Learning mode is disabled\n");
        printf("The color last at the crosshairs will be targeted\n");
        printf("Press l again to learn a different color\n\n");
      }
    }
  
  // HSV image
  cv::Mat hsv;
  cv::cvtColor(*image, hsv, cv::COLOR_BGR2HSV_FULL);

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
    rect = cv::boundingRect(contours[contour_index]);
    cv::rectangle(*image, rect, cv::Scalar(0, 255, 0));
  }

  // Prediction
  cv::Mat1f prediction = kalman.predict();
  int radius = 1e+3 * kalman.errorCovPre.at<float>(0, 0);

  // Show predicted position
  cv::circle(*image, cv::Point(prediction(0, 0), prediction(0, 1)), radius, green, 2);
    
  // Calculate object heading fraction
  float heading = -(((*image).cols/2) - prediction(0, 0))/((*image).cols/2);

  // Sample the object color
  if(learnMode) {
    // Show targeting crosshairs
    cv::line(*image, cvPoint((*image).cols/2, 0), cvPoint((*image).cols/2, (*image).rows/2 - 2), green); //top vertical crosshair
    cv::line(*image, cvPoint((*image).cols/2, (*image).rows/2 + 2), cvPoint((*image).cols/2, (*image).rows), green); //bottom vertical crosshair
    cv::line(*image, cvPoint(0, (*image).rows/2), cvPoint((*image).cols/2 - 2, (*image).rows/2), green); //left horizontal crosshair
    cv::line(*image, cvPoint((*image).cols/2 + 2, (*image).rows/2), cvPoint((*image).cols, (*image).rows/2), green); //right horizontal crosshair

    hsvSample = hsv.at<cv::Vec3b>(cvPoint((*image).cols/2, (*image).rows/2));

    setHSVTrackBarPositions(hsvSample[0], hsvSample[1], hsvSample[2], tolerance);
  }

  displayObjectFollowingInfo(image, heading, hsvSample[0], hsvSample[1], hsvSample[2]);

  rect_area = rect.width * rect.height;

  //Execute drone movement
  if (rect_area > 25000) {
    controlMovements->vx = -1.0;
    moveStatus = false;
  }
  else if (rect_area <= 25000 && rect_area >= 20000) {
    controlMovements->vx = 0;
    moveStatus = false;
  }
  else {
    controlMovements->vx = 1.0;
    moveStatus = true;
  }

  controlMovements->vy = 0;
  controlMovements->vz = 0;
  time_t current_time = time(0);
  double elapsed_time = difftime(current_time, control_ptr->takeoff_time);
  if (elapsed_time < 5){
    controlMovements->vr = 0;
  } else {
    controlMovements->vz = (control_ptr->altitude < 1.0) ? 1.0 : 0.0;
    controlMovements->vr = -(heading * 0.5);
  } 

  return;
}

//Auto set Hue, Saturation, and Value tracking bars
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance) {
  cv::setTrackbarPos("Hue max", "binalized", hue + tolerance);
  cv::setTrackbarPos("Hue min", "binalized", hue - tolerance);

  cv::setTrackbarPos("Saturation max", "binalized", saturation + tolerance);
  cv::setTrackbarPos("Saturation min", "binalized", saturation - tolerance);

  cv::setTrackbarPos("Value max", "binalized", value + tolerance);
  cv::setTrackbarPos("Value min", "binalized", value - tolerance);

}

/*
 */
void ObjectFollowing::displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value) {
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

  putText(*image, headingDisplay, cvPoint(30,120), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, hsvSampleDisplay, cvPoint(30,140), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
  putText(*image, moveStatusDisplay, cvPoint(30,160), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

}
