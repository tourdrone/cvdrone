/*
  Original code from tekkies/CVdrone (get actual address from github)

  Modified by Elliot Greenlee, Caleb Mennen, Jacob Pollack
  COSC 402 Senior Design

  Min Kao Drone Tour
  See readme at (get actual address from github)
*/

#include "ardrone/ardrone.h"
#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char *argv[])
{
  //AR.Drone class
  ARDrone ardrone;
  ardrone.setCamera(1);

  //Display variables
  char vxDisplay[80]; //print buffer for x (forward/reverse) velocity
  char vyDisplay[80]; //print buffer for y (sideways) velocity
  char vzDisplay[80]; //print buffer for z (up/down) velocity
  char vrDisplay[80]; //print buffer for r rotational velocity

  char modeDisplay[80]; //print buffer for flying mode
  char speedDisplay[80]; //print buffer for speed
  char headingDisplay[80]; //print buffer for heading
  char hsvSampleDisplay[80]; //print buffer for learning HSV values

  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  //Drone control
  bool learnMode = false;

  float speed = 0.0;
  double vx = 0.0;
  double vy = 0.0; 
  double vz = 0.0; 
  double vr = 0.0;

  enum FlyingMode {
    Manual,
    ObjectFollowing,
    LineFollowing
  };

  FlyingMode flyingMode = Manual;

  //Object Following Image recognition values
  int learnedHue;
  int learnedSaturation;
  int learnedValue;
  int minH = 0, maxH = 255;
  int minS = 0, maxS = 255;
  int minV = 0, maxV = 255;

  //Sampling time [s]
  const double dt = 1.0;

  //Initializing Message
  printf("Connecting to the drone\n");
  printf("If there is no version number response in the next 10 seconds, please restart the drone and code.\n");
  printf("To disconnect, press the ESC key\n\n");
  fflush(stdout);

  // Initialize
  if (!ardrone.open()) {
    printf("Failed to initialize.\n");
    return -1;
  }

  // XML save data for object following color thresholds
  std::string filename("thresholds.xml");
  cv::FileStorage fs(filename, cv::FileStorage::READ);

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

  // Kalman filter
  cv::KalmanFilter kalman(4, 2, 0);

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

  //Print default command information
  printf("Currently the drone is in manual mode.\n");
  printf("Use the b key for manual mode, the n key for object following, and the m key for line following. The number keys can be used to set a speed. Use spacebar to take off and land, which is required before any control can be executed.\n\n");

  // Main loop
  while (1) {
    // Key input
    int key = cv::waitKey(33);

    //press the escape key to exit
    if (key == 0x1b) {
      break;
    }

    //switch between learning and non-learning mode
    if (key == 'l') {
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

    //switch between flying modes
    if (key == 'b') {
      flyingMode = Manual;
      printf("Manual flying mode is enabled\n");
      printf("Press n for object following and m for line following\n");
      printf("While in manual mode, use q and a for up and down, t and g for forward and backward, and f and h for left and right. Press space to take off.\n\n");
    }
    else if (key == 'n') {
      flyingMode = ObjectFollowing;
      printf("Object Following flying mode is enabled\n");
      printf("Press b for manual and m for line following\n");
      printf("While in object following mode, use l to toggle learning a specific color. Press space to take off after learning a color.\n\n");
    }
    else if (key == 'm') {
      flyingMode = LineFollowing;
      printf("Line Following flying mode is enabled\n");
      printf("Press b for manual and n for object following\n");
      printf("No control for line following yet exists\n\n");
    }

    // Get an image
    cv::Mat image = ardrone.getImage();

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
      cv::Rect rect = cv::boundingRect(contours[contour_index]);
      cv::rectangle(image, rect, cv::Scalar(0, 255, 0));
    }

    // Prediction
    cv::Mat1f prediction = kalman.predict();
    int radius = 1e+3 * kalman.errorCovPre.at<float>(0, 0);
    
    // Calculate object heading fraction
    float heading = -((image.cols/2) - prediction(0, 0))/(image.cols/2);

    // Sample the object color
    if(learnMode) {
      // Show targeting crosshairs
      cv::line(image, cvPoint(image.cols/2, 0), cvPoint(image.cols/2, image.rows/2 - 2), green); //top vertical crosshair
      cv::line(image, cvPoint(image.cols/2, image.rows/2 + 2), cvPoint(image.cols/2, image.rows), green); //bottom vertical crosshair
      cv::line(image, cvPoint(0, image.rows/2), cvPoint(image.cols/2 - 2, image.rows/2), green); //left horizontal crosshair
      cv::line(image, cvPoint(image.cols/2 + 2, image.rows/2), cvPoint(image.cols, image.rows/2), green); //right horizontal crosshair

      cv::Vec3b hsvSample = hsv.at<cv::Vec3b>(cvPoint(image.cols/2, image.rows/2));

      learnedHue = hsvSample[0];
      learnedSaturation = hsvSample[1];
      learnedValue = hsvSample[2];

      //Auto set Hue, Saturation, and Value tracking bars
      cv::setTrackbarPos("Hue max", "binalized", learnedHue+20);
      cv::setTrackbarPos("Hue min", "binalized", learnedHue-20);

      cv::setTrackbarPos("Saturation max", "binalized", learnedSaturation+20);
      cv::setTrackbarPos("Saturation min", "binalized", learnedSaturation-20);
      
      cv::setTrackbarPos("Value max", "binalized", learnedValue+30);
      cv::setTrackbarPos("Value min", "binalized", learnedValue-30);
    }

    // Show predicted position
    cv::circle(image, cv::Point(prediction(0, 0), prediction(0, 1)), radius, green, 2);

    //Speed
    if ((key >= '0') && (key <= '9')) //number keys
    {
      speed = (key-'0')*0.1;
    }

    //Write speed to image
    sprintf(speedDisplay, "Speed = %3.2f", speed);
    putText(image, speedDisplay, cvPoint(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

    //Take off / Landing
    if (key == ' ') { //spacebar
      if (ardrone.onGround()) {
        ardrone.takeoff();
      }
      else {
        ardrone.landing();
      }
    }

    //TODO: print if onground or not
    
    switch (flyingMode) {
      case Manual:
        vx = 0;
        vy = 0;
        vz = 0;
        vr = 0;
        if (key == 't') { vx =  1.0; } //up arrow key
        if (key == 'g') { vx = -1.0; } //down arrow key
        if (key == 'f') { vy =  1.0; } //left arrow key
        if (key == 'h') { vy = -1.0; } //right arrow key
        if (key == 'q') { vz =  1.0; } //q key
        if (key == 'a') { vz = -1.0; } //a key

        sprintf(modeDisplay, "Manual Mode");
        sprintf(vxDisplay, "vx = %3.2f", vx);
        sprintf(vyDisplay, "vy = %3.2f", vy);
        sprintf(vzDisplay, "vz = %3.2f", vz);
        sprintf(vrDisplay, "vr = %3.2f", vr);

        putText(image, vxDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
        putText(image, vyDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
        putText(image, vzDisplay, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
        putText(image, vrDisplay, cvPoint(30,120), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
        break;
      case ObjectFollowing:
        //TODO: set speed to 0.0 when close enough to object
        vx = speed;
        vr = -heading;

        sprintf(modeDisplay, "Object Following Mode");
        sprintf(headingDisplay, "heading = %+3.2f", heading); 
        sprintf(hsvSampleDisplay, "hsvSample = %3d, %3d, %3d", learnedHue, learnedSaturation, learnedValue);

        putText(image, headingDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
        putText(image, hsvSampleDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
        break;
      case LineFollowing:
        sprintf(modeDisplay, "Line Following Mode");
        break;
    }

    putText(image, modeDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

    //Execute drone movement
    ardrone.move3D(vx * speed, vy * speed, vz * speed, vr);
		
    //Display the camera feed
    cv::imshow("camera", image);
  }

  //Save thresholds
  fs.open(filename, cv::FileStorage::WRITE);
  if (fs.isOpened()) {
    cv::write(fs, "H_MAX", maxH);
    cv::write(fs, "H_MIN", minH);
    cv::write(fs, "S_MAX", maxS);
    cv::write(fs, "S_MIN", minS);
    cv::write(fs, "V_MAX", maxV);
    cv::write(fs, "V_MIN", minV);
    fs.release();
  }

  //Close connection to drone
  ardrone.close();

  return 0;
}
