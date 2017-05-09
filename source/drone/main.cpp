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

int main(int argc, char *argv[]) {
  Control *control;
  int wait = 33;

  try {
    //Initialize main control variables and flight modes
    control = new Control();
  } catch (const char *msg) {
    cout << msg << endl;
    return -1;
  }

  // Setting up for constant time
  double time_counter = 0;
  clock_t this_time = clock();
  clock_t last_time = this_time;
  long long executions = 0;
  // Main loop
  while (1) {

    //Time stuff
    this_time = clock();
    time_counter += (double) (this_time - last_time);
    last_time = this_time;
    executions++;

    double time_delay = 1.0 / 25;
    if (time_counter < (time_delay * CLOCKS_PER_SEC)) { //AKA not enough time has passed
      continue;
    }
    time_counter -= time_delay * CLOCKS_PER_SEC;

//    printf("I hit %lli times\n", executions);
    executions = 0;
//    exit(0);


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
