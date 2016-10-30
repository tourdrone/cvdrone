/*
  Original code from tekkies/CVdrone (get actual address from github)

  Modified by Elliot Greenlee, Caleb Mennen, Jacob Pollack
  COSC 402 Senior Design

  Min Kao Drone Tour
  See readme at (get actual address from github)
*/

#include "ardrone/ardrone.h"
#include "control.h"
#include "structures.h"
#include "manual/manual.h"
#include "objectFollowing/objectFollowing.h"
#include "lineFollowing/lineFollowing.h"
#include <iostream>
#include <vector>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
  Control control;

  //Controlling classes
  ObjectFollowing objectFollowing;
  //TODO: ManualDriving manualDriving;
  //TODO: LineFollowing lineFollwing;

  control.initializeDroneControl(&objectFollowing);

  // Main loop
  while (1) {
    control.key = cv::waitKey(33); // Key input

    if (control.detectEscape()) { break; } //Press ESC to close program
    control.detectFlyingMode(); //Press b, n, m to change mode
    control.changeSpeed(); //Press 0-9 to change speed
    control.detectTakeoff(); //Press spacebar to take off

    control.getImage(); //get the image from the camera

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
        //control.velocities = lineFollowingControl();
        break;
    }

    control.overlayControl(); //Display image overlay values

    control.move(); //Send move command to the drone
  }

  //Write hsv values to a file
  objectFollowing.closeObjectFollowing();
  //TODO: closeManual();
  //TODO: closeLineFollowing();

  control.close();

  return 0;
}
