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
    Mat frame, mask;
    cap >> frame; // get a new frame from camera
    cvtColor(frame, edges, CV_BGR2HSV);

    Scalar low, high;
    low = Scalar(30, 0, 240);
    high = Scalar(80, 70, 255);


    inRange(frame, low, high, mask);

    vector <Vec4i> lines;
    HoughLinesP(mask, lines, 1, CV_PI / 180, 100, 100, 10);

    for (size_t i = 0; i < lines.size(); i++) {
      cout << "adding a line\n";
      Vec4i l = lines[i];
      line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
    }


    imshow("edges", frame);
    if (waitKey(30) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return;
}
