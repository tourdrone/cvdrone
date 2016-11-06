/*

*/

#include "lineFollowing.h"
#include "../control.h"

using namespace std;
using namespace cv;

void detect_lines(Mat &original_frame, double scale_factor);


#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

void line_main() {
//  printf("Hello, this is Caleb\n");
//  no this is patric

  VideoCapture cap("../../tests/videos/top_down_1.m4v");
  if (!cap.isOpened())  // check if we succeeded
    return;

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

#pragma clang diagnostic pop

void detect_lines(Mat &original_frame, double scale_factor) {

  Mat hsv;
  Mat mask;
  Mat image;

  resize(original_frame, image, Size(), scale_factor, scale_factor); //Potentially scale down the frame

  cvtColor(image, hsv, CV_BGR2HSV); // Image is now HSV

  double minH = 50;
  double minS = 20;
  double minV = 150;
  double maxH = 125;
  double maxS = 100;
  double maxV = 255;


  Scalar lower(minH, minS, minV);
  Scalar upper(maxH, maxS, maxV);
  inRange(hsv, lower, upper, mask); // Create a mask of only the desired color


  vector<Vec4i> lines;
  HoughLines(mask, lines, 1, CV_PI / 180, 100, 0, 0);
//  HoughLinesP(mask, lines, 1, CV_PI / 180.0, 10, 50, 10); // Find all lines in the image

  printf("Adding in %d lines\n", (int) lines.size());


  //Flipping any backwards lines
  for (int i = 0; i < lines.size(); i++) {
    if (lines[i][1] > CV_PI / 2) {
      lines[i][1] -= CV_PI;
      lines[i][0] *= -1;
    }
    else if (lines[i][1] < ((CV_PI / 2) * -1)) {
      lines[i][1] += CV_PI;
      lines[i][0] *= -1;
    }
  }


  for (size_t i = 0; i < lines.size(); i++) {
    float rho = lines[i][0], theta = lines[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    line(image, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
  }

//  imshow("line_window", image);
  resize(image, original_frame, Size(), 1 / scale_factor, 1 / scale_factor);
}

LineFollowing::LineFollowing(Control *control) {
  namedWindow("line_window", CV_WINDOW_NORMAL);
  control_ptr = control;
  return;
}

void LineFollowing::close() {
  return;
}

void LineFollowing::fly() {

  control_ptr->velocities.vx = 0;
  control_ptr->velocities.vy = 0;
  control_ptr->velocities.vz = 0;
  control_ptr->velocities.vr = 0;

  detect_lines(control_ptr->image, 0.3);

  return;
}
