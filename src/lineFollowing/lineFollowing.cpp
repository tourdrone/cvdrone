/*

*/

#include "lineFollowing.h"


using namespace std;
using namespace cv;

void detect_lines(Mat &frame, double scale_factor);


ControlMovements lineFollowingControl() {

  line_main();

  return ControlMovements();
}

void line_main() {
//  printf("Hello, this is Caleb\n");

  VideoCapture cap("../../tests/videos/top_down_1.m4v");
  if (!cap.isOpened())  // check if we succeeded
    return;

  namedWindow("edges", CV_WINDOW_NORMAL);
//  Mat edges;
  for (; ;) {
    Mat frame;
    cap >> frame; // get a new frame from camera

    detect_lines(frame, .2);


    if (waitKey(30) >= 0) break;

  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return;
}

void detect_lines(Mat &frame, double scale_factor) {
  Mat hsv;
  Mat mask;

  resize(frame, frame, Size(), scale_factor, scale_factor);
  cvtColor(frame, hsv, CV_BGR2HSV);

  double minH = 30;
  double minS = 0;
  double minV = 240;
  double maxH = 80;
  double maxS = 70;
  double maxV = 255;


  Scalar lower(minH, minS, minV);
  Scalar upper(maxH, maxS, maxV);
  inRange(hsv, lower, upper, mask);

//    bitwise_and(frame, frame, frame, mask);


  vector<Vec4i> lines;
  HoughLinesP(mask, lines, 1, CV_PI / 180.0, 10, 50, 10);

  printf("Adding in %d lines\n", (int) lines.size());
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
    // break;
  }
//    if (lines.size() < 1) continue;

  imshow("edges", frame);
}

void LineFollowing::initialize() {
  return;
}

void LineFollowing::close() {
  return;
}

ControlMovements LineFollowing::fly(cv::Mat *image) {
  ControlMovements velocities;

  velocities.vx = 0;
  velocities.vy = 0;
  velocities.vz = 0;
  velocities.vr = 0;


  return velocities;
}
