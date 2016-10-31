/*
*/


#include "manual.h"
#include "../control.h"

/*
*/
ManualFlying::ManualFlying(Control *control) {
  control_ptr = control; 
  return;
}

/*
*/
void ManualFlying::close() {  
  return;
}

/*
*/
void ManualFlying::fly() {
  char key = control_ptr->key;
  
  control_ptr->velocities.vx = 0;
  control_ptr->velocities.vy = 0;
  control_ptr->velocities.vz = 0;
  control_ptr->velocities.vr = 0;

  if (key == 't') { control_ptr->velocities.vx =  1.0; } //t key
  if (key == 'g') { control_ptr->velocities.vx = -1.0; } //g key
  if (key == 'f') { control_ptr->velocities.vy =  1.0; } //f key
  if (key == 'h') { control_ptr->velocities.vy = -1.0; } //h key
  if (key == 'q') { control_ptr->velocities.vz =  1.0; } //q key
  if (key == 'a') { control_ptr->velocities.vz = -1.0; } //a key
  if (key == 'r') { control_ptr->velocities.vr =  1.0; } //r key
  if (key == 'y') { control_ptr->velocities.vr = -1.0; } //y key

  return;
}

