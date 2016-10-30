/*
*/


#include "control.h"

/*
*/
void Control::initialize() {
  //Initializing Message
  printf("Connecting to the drone\n");
  printf("If there is no version number response in the next 10 seconds, please restart the drone and code.\n");
  fflush(stdout);


  flight_log = fopen(flightLog.c_str(), "w");
  green = CV_RGB(0,255,0); 

  //Connect to drone
  if (!ardrone.open()) {
    printf("Failed to initialize.\n");
    //TODO: fix this return -1;
  }

  //Set drone trim on flat surface
  ardrone.setFlatTrim();

  //initialize flying mode code
  manualFlying.initialize();
  objectFollowing.initialize();
  lineFollowing.initialize();

  //Print default command information
  printf("To disconnect, press the ESC key\n\n");
  printf("Currently the drone is in manual mode.\n");
  printf("Use the b key for manual mode, the n key for object following, and the m key for line following. The number keys can be used to set a speed. Use spacebar to take off and land, which is required before any control can be executed.\n\n");
}

ControlMovements Control::fly() {
  //Execute flying instruction code
  switch (flyingMode) {
    case Manual:
      //TODO: Scale these values for normal human control when in manual mode
      velocities = manualFlying.fly(key);
      manualFlying.displayManualInfo(&image, velocities);
      break;
    case ObjectFollow:
      velocities = objectFollowing.fly(&image, key);
      break;
    case LineFollow:
      velocities = lineFollowing.fly();
      break;
  }

  return velocities;
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

/*
  Take off / Landing
*/
void Control::detectTakeoff() {
  if (key == ' ') { //spacebar
    if (ardrone.onGround()) {
      ardrone.takeoff();
      fprintf(flight_log, "TAKEOFF\n");
      //TODO: is this working?
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    else {
      ardrone.landing();
      fprintf(flight_log, "LAND\n");
    }
  }
}

/*
*/
void Control::overlayControl() {

  char modeDisplay[80]; //print buffer for flying mode
  char speedDisplay[80]; //print buffer for speed
  char flyingDisplay[80]; //print buffer for if flying
  char batteryDisplay[80]; //print buffer for battery

  if (flyingMode == Manual) {
    sprintf(modeDisplay, "Manual Mode");
  }
  else if (ObjectFollow) {
    sprintf(modeDisplay, "Object Following Mode");
  }
  else if (LineFollow) {
    sprintf(modeDisplay, "Line Following Mode");
  }

  if (ardrone.onGround()) {
    sprintf(flyingDisplay, "Landed");
  }
  else {
    sprintf(flyingDisplay, "Flying");
  }

  sprintf(speedDisplay, "Speed = %3.2f", speed);
  sprintf(batteryDisplay, "Battery = %d\n", ardrone.getBatteryPercentage());

  //add flying mode to overlay
  putText(image, modeDisplay, cvPoint(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

  //add grounded or flying to overlay
  putText(image, flyingDisplay, cvPoint(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

  //add battery percentage to overlay
  putText(image, batteryDisplay, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

  //add speed to overlay
  putText(image, speedDisplay, cvPoint(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

  cv::imshow("camera", image); //Display the camera feed
}

/*
*/
void Control::move() {
  ardrone.move3D(velocities.vx * speed, velocities.vy * speed, velocities.vz * speed, velocities.vr);

  return;
}


/*
  Close connection to drone
*/
void Control::close() {
  
  //Close flying modes
  manualFlying.close();
  objectFollowing.close();
  lineFollowing.close();

  //close connections with drone
  ardrone.close();
 
  return;
}

