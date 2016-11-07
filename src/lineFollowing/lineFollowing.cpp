/*

 */

#include "lineFollowing.h"
#include "../control.h"

using namespace std;
using namespace cv;

void detect_lines(Mat &original_frame, double scale_factor);
void detect_symbol(Mat &original_frame, double scale_factor);

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
  //    if (lines.size() < 1) continue;

  //  imshow("line_window", image);
  resize(image, original_frame, Size(), 1 / scale_factor, 1 / scale_factor);
}

void MatchingMethod( int, void* );

Mat img, templ, templ_r, templ_l, templ_d, result;
int match_method;
int max_Trackbar = 5;
string img_win = "img win";
string result_win = "result win";
int save = 0;

void detect_symbol(Mat &original_frame, double scale_factor){

  cvtColor(original_frame, img, COLOR_BGR2GRAY);
  namedWindow( img_win, WINDOW_NORMAL);
  resizeWindow( img_win, 500, 500 );


  namedWindow( result_win, WINDOW_NORMAL);
  resizeWindow( result_win, 500, 500 );
  /*
     save++; 

     if(save == 15){
     imwrite( "./savedImage.jpg", original_frame);
     cout << "savedImage\n";
     }
   */

  //	original_frame.copyTo(img);
  //	img = imread( "/home/vflorenc/drone_code/cvdrone/build/linux/imgs/img.jpg", 1);
  Mat templs[4];
  templs[0] = imread( "/home/vflorenc/drone_code/cvdrone/build/linux/templates/template_u.jpg", 1 );
  templs[1] = imread( "/home/vflorenc/drone_code/cvdrone/build/linux/templates/template_d.jpg", 1 );
  templs[2] = imread( "/home/vflorenc/drone_code/cvdrone/build/linux/templates/template_l.jpg", 1 );
  templs[3] = imread( "/home/vflorenc/drone_code/cvdrone/build/linux/templates/template_r.jpg", 1 );

  Scalar colors[4];
  colors[0] = Scalar(0, 255, 0);
  colors[1] = Scalar(255, 0, 0);
  colors[2] = Scalar(0, 0, 255);
  colors[3] = Scalar(0, 0, 0);

  for(int j = 0; j < 4; j++){
    Mat templ = templs[j];

    Mat1f result;
    matchTemplate(original_frame, templs[j], result, TM_CCOEFF_NORMED);

    double thresh = 0.7;
    threshold(result, result, thresh, 1., THRESH_BINARY);

    Mat1b resb;
    result.convertTo(resb, CV_8U, 255);

    vector<vector<Point>> contours;
    findContours(resb, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    for (int i=0; i<(int)contours.size(); ++i)
    {
      Mat1b mask(result.rows, result.cols, uchar(0));
      drawContours(mask, contours, i, Scalar(255), CV_FILLED);

      Point max_point;
      double max_val;
      minMaxLoc(result, NULL, &max_val, NULL, &max_point, mask);

      rectangle(original_frame, Rect(max_point.x, max_point.y, templ.cols, templ.rows), colors[j], 2);
    }


  }

  return;

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

  detect_symbol(control_ptr->image, 0.3);
  //detect_lines(control_ptr->image, 0.3);

  return;
}
