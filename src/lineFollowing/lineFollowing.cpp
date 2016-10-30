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
//  printf("Hello, this is Caleb\n");

  VideoCapture cap("../../tests/videos/top_down_4.m4v");
  if (!cap.isOpened())  // check if we succeeded
    return;

  namedWindow("edges", CV_WINDOW_NORMAL);
  Mat edges;
  for (; ;) {
    Mat frame;
    cap >> frame; // get a new frame from camera
    cvtColor(frame, edges, CV_BGR2GRAY);
    GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
    Canny(edges, edges, 0, 30, 3);
    imshow("edges", edges);
    if (waitKey(30) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return;
}
