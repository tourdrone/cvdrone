/*

*/

#include "lineFollowing.h"
#include "../control.h"

using namespace std;
using namespace cv;


void compress_lines(vector<Vec2f> &condensed, const vector<Vec2f> &tmp_list);

void draw_lines(Mat &image, const vector<Vec2f> &lines);

vector<Vec2f> condense_lines(vector<Vec2f> lines);


void LineFollowing::detect_lines(Mat &original_frame) {

  Mat hsv;
  Mat mask;


  cvtColor(original_frame, hsv, CV_BGR2HSV); // Image is now HSV

  Scalar lower(minH, minS, minV);
  Scalar upper(maxH, maxS, maxV);
  inRange(hsv, lower, upper, mask); // Create a mask of only the desired color
  Canny(mask, mask, 50, 200, 3);

  vector<Vec2f> lines;
  HoughLines(mask, lines, 1, CV_PI / 180, 100, 0, 0);

  printf("Adding in %d lines\n", (int) lines.size());

  found_lines = condense_lines(lines);
  draw_lines(original_frame, found_lines);


}

vector<Vec2f> condense_lines(vector<Vec2f> lines) {
  vector<Vec2f> condensed;
  vector<Vec2f> tmp_list;
  for (int i = 0; i < lines.size(); i++) {
    if (lines[i][1] > CV_PI / 2) {
      lines[i][1] -= CV_PI;
      lines[i][0] *= -1;
    }
    else if (lines[i][1] < ((CV_PI / 2) * -1)) {
      lines[i][1] += CV_PI;
      lines[i][0] *= -1;
    }

    //put in order of theta, rho
    swap(lines[i][0], lines[i][1]);
  }

  //Order from least to greatest theta
  sort(lines.begin(), lines.end(),
       [](const Vec2f &a, const Vec2f &b) {
         return a[0] < b[0];
       });

  while (!lines.empty()) {
    Vec2f to_manipulate = lines.front();
    lines.erase(lines.begin());

    if (tmp_list.empty()) {
      tmp_list.push_back(to_manipulate);
      continue;
    } else {
      if (abs(to_manipulate[0] - tmp_list.front()[0]) < 20 * (CV_PI / 180.0)) {
        //The angles are similar
        if (abs(to_manipulate[1] - tmp_list.front()[1]) < 50) {
          //the distances are similar
          tmp_list.push_back(to_manipulate);
          continue;
        }
      } else {
        //Need to clear out the tmp_list
        compress_lines(condensed, tmp_list);

        tmp_list.clear();
        tmp_list.push_back(to_manipulate);

      }
    }
  }
  if (!tmp_list.empty()) {
    compress_lines(condensed, tmp_list);
  }
  return condensed;
}

void draw_lines(Mat &image, const vector<Vec2f> &lines) {
  for (size_t i = 0; i < lines.size(); i++) {
    float theta = lines[i][0], rho = lines[i][1];
    // float rho = lines[i][0], theta = lines[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    line(image, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
  }
}

void compress_lines(vector<Vec2f> &condensed, const vector<Vec2f> &tmp_list) {
  Vec2f new_point;
  float angle_sum = 0;
  float rho_sum = 0;
  for (int j = 0; j < tmp_list.size(); ++j) {
    angle_sum += tmp_list[j][0];
    rho_sum += tmp_list[j][1];
  }
  angle_sum /= tmp_list.size();
  rho_sum /= tmp_list.size();

  new_point[0] = angle_sum;
  new_point[1] = rho_sum;

  condensed.push_back(new_point);
}

LineFollowing::LineFollowing(Control *control) {
  namedWindow("line_window", CV_WINDOW_NORMAL);
  control_ptr = control;
  minH = 50;
  minS = 20;
  minV = 150;
  maxH = 125;
  maxS = 100;
  maxV = 255;
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

  detect_lines(control_ptr->image);


  return;
}
