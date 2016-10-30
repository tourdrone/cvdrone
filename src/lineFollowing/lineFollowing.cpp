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
//  Mat edges;
  for (; ;) {
    Mat mask;
    Mat frame, hsv;
    cap >> frame; // get a new frame from camera

    cvtColor(frame, hsv, CV_BGR2HSV);

    double minH = 30;
    double minS = 0;
    double minV = 240;
    double maxH = 80;
    double maxS = 70;
    double maxV = 255;

    cv::Scalar lower(minH, minS, minV);
    cv::Scalar upper(maxH, maxS, maxV);
    cv::inRange(hsv, lower, upper, mask);

//    bitwise_and(frame, frame, frame, mask);



    erode(mask, mask, Mat(), Point(-1,-1), 5);
    medianBlur(mask, mask, 5);

    vector <Vec4i> lines;
    HoughLinesP(mask, lines, 1, CV_PI / 180.0, 10, 100, 10);

    printf("Adding in %d lines\n", (int) lines.size());
    for (size_t i = 0; i < lines.size(); i++) {
      Vec4i l = lines[i];
      line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
      // break;
    }
//    if (lines.size() < 1) continue;

    imshow("edges", mask);
    if (waitKey(30) >= 0) break;

  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return;
}
