/*
*/


#include "manual.h"

/*
*/
void ManualFlying::initialize() {
  return;
}

/*
*/
void ManualFlying::close() {  
  return;
}

/*
*/
ControlMovements ManualFlying::fly(int key) {
  ControlMovements velocities;
  
  velocities.vx = 0;
  velocities.vy = 0;
  velocities.vz = 0;
  velocities.vr = 0;

  if (key == 't') { velocities.vx =  1.0; } //t key
  if (key == 'g') { velocities.vx = -1.0; } //g key
  if (key == 'f') { velocities.vy =  1.0; } //f key
  if (key == 'h') { velocities.vy = -1.0; } //h key
  if (key == 'q') { velocities.vz =  1.0; } //q key
  if (key == 'a') { velocities.vz = -1.0; } //a key
  if (key == 'r') { velocities.vr =  1.0; } //r key
  if (key == 'y') { velocities.vr = -1.0; } //y key

  return velocities;
}

