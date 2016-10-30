/*
  Original code from tekkies/CVdrone (get actual address from github)

  Modified by Elliot Greenlee, Caleb Mennen, Jacob Pollack
  COSC 402 Senior Design

  Min Kao Drone Tour
  See readme at (get actual address from github)
*/

#include "ardrone/ardrone.h"
#include "structures.h"
#include "manual/manual.h"
#include "objectFollowing/objectFollowing.h"
#include "lineFollowing/lineFollowing.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

using namespace std;

class Control {
  public:
    const string flightLog = "flight_log.txt";

    //AR.Drone class
    ARDrone ardrone;

    cv::Mat image;

    int key;
    FlyingMode flyingMode = Manual;
    double speed = 0.0;
    int batteryPercentage;
    bool flying;

    ControlMovements velocities;

    FILE *flight_log;

    void initializeDroneControl(ObjectFollowing *objectFollowing);
    void detectFlyingMode();
    bool detectEscape();
    void changeSpeed();
    void getImage();

    void overlayControl();
};

/*
*/
void Control::initializeDroneControl(ObjectFollowing *objectFollowing) {
  //Initializing Message
  printf("Connecting to the drone\n");
  printf("If there is no version number response in the next 10 seconds, please restart the drone and code.\n");
  fflush(stdout);

  // Initialize
  if (!ardrone.open()) {
    printf("Failed to initialize.\n");
    //TODO: fix this return -1;
  }

  //Set drone on flat surface and initialize
  ardrone.setFlatTrim();

  //initialize object following code
  objectFollowing->initializeObjectFollowing();
  flight_log = fopen(flightLog.c_str(), "w");

  //Print default command information
  printf("To disconnect, press the ESC key\n\n");
  printf("Currently the drone is in manual mode.\n");
  printf("Use the b key for manual mode, the n key for object following, and the m key for line following. The number keys can be used to set a speed. Use spacebar to take off and land, which is required before any control can be executed.\n\n");
}

/*
  Detect ESC key press and
*/
bool Control::detectEscape() {
  //Escape key
  if (key == 0x1b) { return true; }
  return false;
}

/*
*/
void Control::changeSpeed() {
  if ((key >= '0') && (key <= '9')) //number keys
  {
    speed = (key-'0')*0.1;
  }
}

/*
*/
void Control::getImage() {
//Get an image
  image = ardrone.getImage();
}

/*
  Switch between flying modes
*/
void Control::detectFlyingMode() {

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
}

void Control::overlayControl() {
  //TODO: move this so it isnt called every time
  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  char modeDisplay[80]; //print buffer for flying mode
  char speedDisplay[80]; //print buffer for speed

  if (flyingMode == Manual) {
    sprintf(modeDisplay, "Manual Mode");
  }
  else if (flyingMode == ObjectFollow) {
    sprintf(modeDisplay, "Object Following Mode");
  }
  else if (flyingMode == LineFollow) {
    sprintf(modeDisplay, "Line Following Mode");
  }

  sprintf(speedDisplay, "Speed = %3.2f", speed);

  //add speed to overlay
  putText(image, speedDisplay, cvPoint(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

  //add flying mode to overlay
  putText(image, modeDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

  cv::imshow("camera", image); //Display the camera feed
}

int main(int argc, char *argv[])
{
  Control control;

  //Controlling classes
  ObjectFollowing objectFollowing;
  //TODO: ManualDriving manualDriving;
  //TODO: LineFollowing lineFollwing;

  //Display variables
  char flyingDisplay[80]; //print buffer for if flying

  cv::Scalar green = CV_RGB(0,255,0); //putText color value

  line_main();
  return 1;
  control.initializeDroneControl(&objectFollowing);

  // Main loop
  while (1) {
    control.key = cv::waitKey(33); // Key input

    if (control.detectEscape()) { break; } //Press ESC to close program

    control.detectFlyingMode(); //Press b, n, m to change mode

    control.changeSpeed(); //Press 0-9 to change speed

    control.getImage(); //get the image from the camera


    //Take off / Landing
    if (control.key == ' ') { //spacebar
      if (control.ardrone.onGround()) {
        control.ardrone.takeoff();
	fprintf(control.flight_log, "TAKEOFF\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      }
      else {
        control.ardrone.landing();
	fprintf(control.flight_log, "LAND\n");
      }
    }

    //TODO:Write battery percentage to screen
    printf("%d\n", control.ardrone.getBatteryPercentage());


    //Write if grounded or flying to image
    if (control.ardrone.onGround()) {
      sprintf(flyingDisplay, "Landed");
    }
    else {
      sprintf(flyingDisplay, "Flying");
    }
    putText(control.image, flyingDisplay, cvPoint(200,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

    switch (control.flyingMode) {
      case Manual:
        //TODO: Allow user to set camera mode in Manual
        //TODO: Change 0/1 to CONSTANTS of 0 = front, 1 = bottom

        //TODO: Scale these values for normal human control when in manual mode
        control.velocities = manualMovement(control.key);


        //TODO: Move this into manualMovement(control.key) function
        displayManualInfo(&(control.image), control.velocities);

        break;

      case ObjectFollow:
        control.velocities = objectFollowing.detectObject(control.image, control.key);

        break;

      case LineFollow:
          //TODO implement for real
        control.velocities = lineFollowingControl();
        break;
    }

    control.overlayControl();

    control.ardrone.move3D(control.velocities.vx * control.speed, control.velocities.vy * control.speed, control.velocities.vz * control.speed, control.velocities.vr);

  }

  //Write hsv values to a file
  objectFollowing.closeObjectFollowing();
  //TODO: closeManual();
  //TODO: closeLineFollowing();

  //Close connection to drone
  control.ardrone.close();

  return 0;
}
