/*

*/

#include "lineFollowing.h"

using namespace std;
using namespace cv;

ControlMovements lineFollowingControl() {

  line_main();

  return ControlMovements();
}

void line_main() {
  printf("Hello, this is Caleb\n");
  VideoCapture cap("videos/top_down_4.m4v");
  if (!cap.isOpened())  // check if we succeeded
    return;
}
