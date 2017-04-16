#ifndef CONTROL_HEADER
#define CONTROL_HEADER 


/*
*/
struct ControlMovements {
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double vr = 0.0;
};

/*
*/
enum FlyingMode {
  Manual,
  ObjectFollow,
  LineFollow,
  Recording
};

#endif