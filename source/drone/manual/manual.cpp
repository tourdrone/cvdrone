/*
*/


#include "manual.h"
#include "../control.h"

directions directions;

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

  if (key == directions.forwards) { control_ptr->velocities.vx = 1.0; } //t key forwards
  if (key == directions.backwards) { control_ptr->velocities.vx = -1.0; } //g key backwards
  if (key == directions.left) { control_ptr->velocities.vy = 1.0; } //f key left
  if (key == directions.right) { control_ptr->velocities.vy = -1.0; } //h key right
  if (key == directions.up) { control_ptr->velocities.vz = 1.0; } //q key
  if (key == directions.down) { control_ptr->velocities.vz = -1.0; } //a key
  if (key == directions.counterClockwise) { control_ptr->velocities.vr = 1.0; } //r key
  if (key == directions.clockwise) { control_ptr->velocities.vr = -1.0; } //y key

  return;
}

