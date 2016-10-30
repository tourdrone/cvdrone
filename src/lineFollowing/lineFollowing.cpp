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
    Mat mask, hsv_mask;
    Mat frame, hsv;
    cap >> frame; // get a new frame from camera

    cvtColor(frame, frame, CV_BGR2HSV);

    double minH = 0;
    double minS = 0;
    double minV = 0;
    double maxH = 255;
    double maxS = 255;
    double maxV = 255;

    cv::Scalar lower(minH, minS, minV);
    cv::Scalar upper(maxH, maxS, maxV);
    cv::inRange(frame, lower, upper, mask);
    
    //    low = Scalar(30, 0, 240);
//    high = Scalar(80, 70, 255);

//    inRange(hsv, Scalar(100, 0, 0), Scalar(255, 255, 255), mask);

    bitwise_and(frame, frame, frame, mask);

//    cvtColor(mask, hsv_mask, CV_HS);

    vector <Vec4i> lines;
//    HoughLinesP(mask, lines, 1, CV_PI / 180, 100, 100, 10);

    printf("Adding in %d lines\n", (int) lines.size());
    for (size_t i = 0; i < lines.size(); i++) {
      Vec4i l = lines[i];
      line(hsv_mask, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
    }


    imshow("edges", frame);
    if (waitKey(30) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return;
}
