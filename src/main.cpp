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

class Control {
  public:
    //AR.Drone class
    ARDrone ardrone;
    
    int key;
    FlyingMode flyingMode = Manual;
    double speed;
    int batteryPercentage;
    bool flying;

    ControlMovements controlMovements;

    FILE *flight_log;

    void detectFlyingMode();
};

void Control::detectFlyingMode() {

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
}

int main(int argc, char *argv[])
{
  Control control;

  //Controlling classes
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
  ControlMovements controlMovements;

  //Initializing Message
  printf("Connecting to the drone\n");
  printf("If there is no version number response in the next 10 seconds, please restart the drone and code.\n");
  printf("To disconnect, press the ESC key\n\n");
  fflush(stdout);

  // Initialize
  if (!(control.ardrone.open())) {
    printf("Failed to initialize.\n");
    return -1;
  }

  //Set drone on flat surface and initialize
  control.ardrone.setFlatTrim();

  //initialize object following code
  objectFollowing.initializeObjectFollowing();

  //Print default command information
  printf("Currently the drone is in manual mode.\n");
  printf("Use the b key for manual mode, the n key for object following, and the m key for line following. The number keys can be used to set a speed. Use spacebar to take off and land, which is required before any control can be executed.\n\n");

  flight_log = fopen("flight_log.txt", "w");

  // Main loop
  while (1) {
    control.key = cv::waitKey(33); // Key input

    if (control.key == 0x1b) { break; } //press the escape key to exit

    //TODO:Write battery percentage to screen
    printf("%d\n", control.ardrone.getBatteryPercentage());

    control.detectFlyingMode();

    // Get an image
    cv::Mat image = control.ardrone.getImage();
    
    //Speed
    if ((control.key >= '0') && (control.key <= '9')) //number keys
    {
      speed = (control.key-'0')*0.1;
    }

    //Write speed to image
    sprintf(speedDisplay, "Speed = %3.2f", speed);
    putText(image, speedDisplay, cvPoint(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

    //Take off / Landing
    if (control.key == ' ') { //spacebar
      if (control.ardrone.onGround()) {
        control.ardrone.takeoff();
	fprintf(flight_log, "TAKEOFF\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      }
      else {
        control.ardrone.landing();
	fprintf(flight_log, "LAND\n");
      }
    }

    //Write if grounded or flying to image
    if (control.ardrone.onGround()) {
      sprintf(flyingDisplay, "Landed");
    }
    else {
      sprintf(flyingDisplay, "Flying");
    }
    putText(image, flyingDisplay, cvPoint(200,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
    
    switch (control.flyingMode) {
      case Manual:
        //TODO: Allow user to set camera mode in Manual
        //TODO: Change 0/1 to CONSTANTS of 0 = front, 1 = bottom

        //TODO: Scale these values for normal human control when in manual mode
        controlMovements = manualMovement(control.key);

        sprintf(modeDisplay, "Manual Mode");

        //TODO: Move this into manualMovement(control.key) function
        displayManualInfo(&image, controlMovements);

        break;

      case ObjectFollow:

        controlMovements = objectFollowing.detectObject(image, control.key);

        sprintf(modeDisplay, "Object Following Mode");
        break;

      case LineFollow:
        sprintf(modeDisplay, "Line Following Mode");
        break;
    }

    putText(image, modeDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);


    control.ardrone.move3D(controlMovements.vx * speed, controlMovements.vy * speed, controlMovements.vz * speed, controlMovements.vr);

    //Display the camera feed
    cv::imshow("camera", image);
  }

  //Write hsv values to a file
  objectFollowing.closeObjectFollowing();
  //TODO: closeManual();
  //TODO: closeLineFollowing();

  //Close connection to drone
  control.ardrone.close();

  return 0;
}
