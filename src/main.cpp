/*
  Original code from tekkies/CVdrone (get actual address from github)

  Modified by Elliot Greenlee, Caleb Mennen, Jacob Pollack
  COSC 402 Senior Design

  Min Kao Drone Tour
  See readme at (get actual address from github)
*/

#include "ardrone/ardrone.h"
#include "structures.h"
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

  //Control classes
  ObjectFollowing objectFollowing;
  //TODO: ManualDriving manualDriving;
  //TODO: LineFollowing lineFollwing;

  //Display variables
  FILE *flight_log;

  char modeDisplay[80]; //print buffer for flying mode
  char flyingDisplay[80]; //print buffer for if flying
  char speedDisplay[80]; //print buffer for speed

  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  //Drone control

  float speed = 0.0;
  double vx = 0.0;
  double vy = 0.0; 
  double vz = 0.0; 
  double vr = 0.0;
  ControlMovements controlMovements;

  FlyingMode flyingMode = Manual;

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

  //initialize object following code
  objectFollowing.initializeObjectFollowing();

  //Print default command information
  printf("Currently the drone is in manual mode.\n");
  printf("Use the b key for manual mode, the n key for object following, and the m key for line following. The number keys can be used to set a speed. Use spacebar to take off and land, which is required before any control can be executed.\n\n");

  flight_log = fopen("flight_log.txt", "w");

  // Main loop
  while (1) {
    int key = cv::waitKey(33); // Key input

    if (key == 0x1b) { break; } //press the escape key to exit


    //TODO:FlyingMode detectFlyingMode(ardrone, key) {
    //}

    //switch between flying modes
    if (key == 'b') {
      flyingMode = Manual;
      ardrone.setCamera(0);
      printf("Manual flying mode is enabled\n");
      printf("Press n for object following and m for line following\n");
      printf("While in manual mode, use q and a for up and down, t and g for forward and backward, and f and h for left and right. Press space to take off.\n\n");
    }
    else if (key == 'n') {
      flyingMode = ObjectFollow;
      ardrone.setCamera(0);
      printf("Object Following flying mode is enabled\n");
      printf("Press b for manual and m for line following\n");
      printf("While in object following mode, use l to toggle learning a specific color. Press space to take off after learning a color.\n\n");
    }
    else if (key == 'm') {
      flyingMode = LineFollow;
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
        //TODO: Allow user to set camera mode in Manual
        //TODO: Change 0/1 to CONSTANTS of 0 = front, 1 = bottom

        //TODO: Scale these values for normal human control when in manual mode
        controlMovements = manualMovement(key);

        sprintf(modeDisplay, "Manual Mode");

        //TODO: Move this into manualMovement(key) function
        displayManualInfo(&image, controlMovements);

        vx = controlMovements.vx;
        vy = controlMovements.vy;
        vz = controlMovements.vz;
        vr = controlMovements.vr;
        break;

      case ObjectFollow:

        controlMovements = objectFollowing.detectObject(image, key);

        vx = controlMovements.vx;
        vy = controlMovements.vy;
        vz = controlMovements.vz;
        vr = controlMovements.vr;

        sprintf(modeDisplay, "Object Following Mode");
        break;

      case LineFollow:
        sprintf(modeDisplay, "Line Following Mode");
        break;
    }

    putText(image, modeDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

    fprintf(flight_log, "ardrone.move3D(vx=%f, vy=%f, vz=%f, vr=%f)\n", (speed), (vy * speed), (vz * speed), vr);

    ardrone.move3D(vx * speed, vy * speed, vz * speed, vr);

    //Display the camera feed
    cv::imshow("camera", image);
  }

  //Write hsv values to a file
  objectFollowing.closeObjectFollowing();
  //TODO: closeManual();
  //TODO: closeLineFollowing();

  //Close connection to drone
  ardrone.close();

  return 0;
}
