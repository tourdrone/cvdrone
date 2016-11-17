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
#include <iostream>
#include <vector>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
  Control *control;
  int wait = 33;

  try {
    //Initialize main control variables and flight modes
    control = new Control();
  }catch (const char* msg){
    cout << msg << endl;
    return -1;
  }

  // Main loop
  while (1) {
    //Detect user key input and end loop if ESC is pressed
    if (!control->getKey(wait)) break;

    control->detectFlyingMode(); //b, n, m to change mode
    control->changeSpeed(); //0-9 to change speed
    control->detectTakeoff(); //spacebar to take off

    //Get the image from the camera
    control->getImage();

    //Run drone control
    control->fly();

    //Display image overlay values
    control->overlayControl(); 

    //Send move command to the drone
    control->move(); 
  }

  //Close flying modes and drone connection
  control->close();

  return 0;
}
