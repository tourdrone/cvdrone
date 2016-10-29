/*
  Original code from tekkies/CVdrone (get actual address from github)

  Modified by Elliot Greenlee, Caleb Mennen, Jacob Pollack
  COSC 402 Senior Design

  Min Kao Drone Tour
  See readme at (get actual address from github)
*/

#include "ardrone/ardrone.h"
#include "objectFollowing/objectFollowing.h"
#include "manual/manual.h"
#include "lineFollowing/lineFollowing.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

using namespace std;

int main(int argc, char *argv[])
{
  //AR.Drone class
  ARDrone ardrone;

  //Display variables

  char modeDisplay[80]; //print buffer for flying mode
  char flyingDisplay[80]; //print buffer for if flying
  char speedDisplay[80]; //print buffer for speed

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

  //Object Following variables
  int minH = 0, maxH = 255;
  int minS = 0, maxS = 255;
  int minV = 0, maxV = 255;
  int rect_area = 0;
  bool moveStatus = false;
  const char *moveStatuses[2] = {"STOP", "GO"};
  float heading = 0;
  cv::Rect rect;

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

  cv::KalmanFilter kalman(4, 2, 0);

  //Open and read Thresholds file for object following hsv values
  //Create a window for object following hsv and binalization
  initializeObjectFollowing(&kalman, &maxH, &minH, &maxS, &minS, &maxV, &minV);

  //Print default command information
  printf("Currently the drone is in manual mode.\n");
  printf("Use the b key for manual mode, the n key for object following, and the m key for line following. The number keys can be used to set a speed. Use spacebar to take off and land, which is required before any control can be executed.\n\n");

  FILE *flight_log;
  flight_log = fopen("flight_log.txt", "w");

  // Main loop
  while (1) {
    speed = 0.0;
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
      //TODO: Allow user to set camera mode in Manual
      //TODO: Change 0/1 to CONSTANTS of 0 = front, 1 = bottom
      ardrone.setCamera(0);
      printf("Manual flying mode is enabled\n");
      printf("Press n for object following and m for line following\n");
      printf("While in manual mode, use q and a for up and down, t and g for forward and backward, and f and h for left and right. Press space to take off.\n\n");
    }
    else if (key == 'n') {
      flyingMode = ObjectFollowing;
      ardrone.setCamera(0);
      printf("Object Following flying mode is enabled\n");
      printf("Press b for manual and m for line following\n");
      printf("While in object following mode, use l to toggle learning a specific color. Press space to take off after learning a color.\n\n");
    }
    else if (key == 'm') {
      flyingMode = LineFollowing;
      ardrone.setCamera(1);
      printf("Line Following flying mode is enabled\n");
      printf("Press b for manual and n for object following\n");
      printf("No control for line following yet exists\n\n");
    }

    // Get an image
    cv::Mat image = ardrone.getImage();
    
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
	fprintf(flight_log, "TAKEOFF\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      }
      else {
        ardrone.landing();
	fprintf(flight_log, "LAND\n");
      }
    }

    //Write if grounded or flying to image
    if (ardrone.onGround()) {
      sprintf(flyingDisplay, "Landed");
    }
    else {
      sprintf(flyingDisplay, "Flying");
    }
    putText(image, flyingDisplay, cvPoint(200,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
    
    switch (flyingMode) {
      case Manual:
        //TODO: Scale these values for normal human control when in manual mode
        vx = 0;
        vy = 0;
        vz = 0;
        vr = 0;
        if (key == 't') { vx =  1.0; } //t key
        if (key == 'g') { vx = -1.0; } //g key
        if (key == 'f') { vy =  1.0; } //f key
        if (key == 'h') { vy = -1.0; } //h key
        if (key == 'q') { vz =  1.0; } //q key
        if (key == 'a') { vz = -1.0; } //a key
        //TODO: Verify if these vr values should be negative
        if (key == 'r') { vr =  1.0; } //r key
        if (key == 'y') { vr = -1.0; } //y key


        sprintf(modeDisplay, "Manual Mode");
        displayManualInfo(&image, vx, vy, vz, vr);
        break;

      case ObjectFollowing:
        heading = detectObject(image, kalman, minH, maxH, minS, maxS, minV, maxV, learnMode, moveStatus, &rect);

        rect_area = rect.width * rect.height;

        //Execute drone movement

        //printf("rect.width: %d, rect.height: %d, rect.area: %d mode: %d\n", rect.width, rect.height, rect_area, flyingMode);

        if (rect_area > 10000) {
          speed = -0.2;
          vx = 0;
          
          moveStatus = false;
        }
        else {
          moveStatus = true;
        }

        vx = speed;
        vr = -heading;

        sprintf(modeDisplay, "Object Following Mode");
        break;

      case LineFollowing:
        sprintf(modeDisplay, "Line Following Mode");
        break;
    }

    putText(image, modeDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

    fprintf(flight_log, "%s ardrone.move3D(vx=%f, vy=%f, vz=%f, vr=%f) || rect_area = %d\n",
        moveStatuses[moveStatus], (speed), (vy * speed), (vz * speed), vr, rect_area);

    ardrone.move3D(speed, vy * speed, vz * speed, vr);

    //Display the camera feed
    cv::imshow("camera", image);
  }

  //Write hsv values to a file
  closeObjectFollowing(maxH, minH, maxS, minS, maxV, minV);
  //TODO: create close manual and close line following functions

  //Close connection to drone
  ardrone.close();

  return 0;
}


