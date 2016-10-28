#include "ardrone/ardrone.h"
#include <iostream>
#include <vector>

using namespace std;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  //AR.Drone class
  ARDrone ardrone;

  //Display variables
  char vxDisplay[80]; //print buffer for x (forward/reverse) velocity
  char vyDisplay[80]; //print buffer for y (sideways) velocity
  char vzDisplay[80]; //print buffer for z (up/down) velocity
  char vrDisplay[80]; //print buffer for r rotational velocity

  char speedDisplay[80]; //print buffer for speed
  char headingDisplay[80]; //print buffer for heading
  char hsvSampleDisplay[80]; //print buffer for learning HSV values

  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  //Drone control
  bool autonomous = false;
  bool learnMode = false;

  float speed = 0.0;
  double vx = 0.0;
  double vy = 0.0; double vz = 0.0; 
  double vr = 0.0;

  //Image recognition values
  int learnedHue;
  int learnedSaturation;
  int learnedValue;
  int minH = 0, maxH = 255;
  int minS = 0, maxS = 255;
  int minV = 0, maxV = 255;

  //Sampling time [s]
  const double dt = 1.0;

  printf("Connecting to the drone\n");
  printf("If there is no version number response in the next 10 seconds, please restart the drone and code.\n");
  printf("To disconnect, press the ESC key\n");
  fflush(stdout);

  // Initialize
  if (!ardrone.open()) {
    printf("Failed to initialize.\n");
    return -1;
  }

  // XML save data
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
  cv::createTrackbar("H max", "binalized", &maxH, 255);
  cv::createTrackbar("H min", "binalized", &minH, 255);
  cv::createTrackbar("S max", "binalized", &maxS, 255);
  cv::createTrackbar("S min", "binalized", &minS, 255);
  cv::createTrackbar("V max", "binalized", &maxV, 255);
  cv::createTrackbar("V min", "binalized", &minV, 255);
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


  // Main loop
  while (1) {
    // Key input
    int key = cv::waitKey(33);
    printf("%c\n", (char)key);

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
        printf("Press l again to turn off learning mode\n");
      }
      else {
        printf("Learning mode is disabled\n");
        printf("The color last at the crosshairs will be targeted\n");
        printf("Press l again to learn a different color\n");
      }
    }

    //switch between autonomous and manual flying mode
    if (key == 'm') {
      autonomous = !autonomous;
      if (autonomous) {
        printf("Autonomous flying mode is enabled\n");
	printf("Press m again to switch to manual flying mode\n");
      }
      else {
        printf("Manual flying mode is enabled\n");
        printf("Press m again to switch to automatic flying mode\n");
      }
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
      cv::setTrackbarPos("H max", "binalized", learnedHue+20);
      cv::setTrackbarPos("H min", "binalized", learnedHue-20);

      cv::setTrackbarPos("S max", "binalized", learnedSaturation+20);
      cv::setTrackbarPos("S min", "binalized", learnedSaturation-20);
      
      cv::setTrackbarPos("V max", "binalized", learnedValue+30);
      cv::setTrackbarPos("V min", "binalized", learnedValue-30);
    }

    // Show predicted position
    cv::circle(image, cv::Point(prediction(0, 0), prediction(0, 1)), radius, green, 2);
    
    //TODO:Switch control between manual and automatic mode

    //Speed
    if ((key >= '0') && (key <= '9')) 
    {
      speed = (key-'0')*0.1;
    }

    // Auto-follow
    if (autonomous) {
      //TODO: set speed to 0.0 when close enough to object
      vx = speed;
      vr = -heading;
    }
    //Manual flying mode
    else {
      if (key == 0x260000) { vx =  1.0 * speed; } //up arrow key
      if (key == 0x280000) { vx = -1.0; } //down arrow key
      if (key == 0x250000) { vr =  1.0; } //left arrow key
      if (key == 0x270000) { vr = -1.0; } //right arrow key
      if (key == 'q') { vz =  1.0; } //q key
      if (key == 'a') { vz = -1.0; } //a key
    }

    ardrone.move3D(vx, vy, vz, vr);

    // Take off / Landing to specific height
    if (key == ' ') {
      if (ardrone.onGround()) {
        ardrone.takeoff();
      }
      else {
        ardrone.landing();
      }
    }
		
    //Display the camera feed
    cv::imshow("camera", image);

    //Write control information over the image
    sprintf(speedDisplay, "speed = %3.2f", speed);
    putText(image, speedDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
    
    //If running autonomously
    if (autonomous) {
      sprintf(headingDisplay, "heading = %+3.2f", heading); 
      sprintf(hsvSampleDisplay, "hsvSample = %3d, %3d, %3d", learnedHue, learnedSaturation, learnedValue);

      putText(image, headingDisplay, cvPoint(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
      putText(image, hsvSampleDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
    }
    //If running in manual mode
    else {
      sprintf(vxDisplay, "vx = %3.2f", vx);
      sprintf(vyDisplay, "vy = %3.2f", vy);
      sprintf(vzDisplay, "vz = %3.2f", vz);
      sprintf(vrDisplay, "vr = %3.2f", vr);

      putText(image, vxDisplay, cvPoint(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
      putText(image, vyDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
      putText(image, vzDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
      putText(image, vrDisplay, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
    }
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
