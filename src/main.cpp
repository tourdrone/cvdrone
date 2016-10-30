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
  ManualFlying manualFlying;
  //TODO: LineFollowing lineFollwing;

  control.initializeDroneControl(&objectFollowing, &manualFlying);

  // Main loop
  while (1) {
    //Detect user key input
    control.key = cv::waitKey(33); 

    if (control.detectEscape()) { break; } //esc to close program
    control.detectFlyingMode(); //b, n, m to change mode
    control.changeSpeed(); //0-9 to change speed
    control.detectTakeoff(); //spacebar to take off

    //Get the image from the camera
    control.getImage();

    //Execute flying instruction code
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

    //Display image overlay values
    control.overlayControl(); 

    control.move(); //Send move command to the drone
  }

  //TODO: closeLineFollowing();
  control.close(&objectFollowing, &manualFlying);

  return 0;
}
