//
// Created by Caleb on 11/15/16.
//

#include "lineFollowing.h"
#include "line_utilities.h"



cv::Vec2f flip_line(cv::Vec2f line) {
  line[0] -= deg2rad(180);
  line[1] *= -1;
  return line;
}

double deg2rad(double deg) {
  return deg * (CV_PI / 180.0);
}

double rad2deg(double rad) {
  return rad * (180.0 / CV_PI);
}

double LineFollowing::distance_from_center(float rho, float theta, double width, double height) {
  // printf("width: %f height: %f\n", width, height);
  theta = -1 * (theta);
  double rh = rho * cos(theta);
  double rv = rho * sin(theta);
  // rh = abs(rh);
  // rv *= -1;
  double excess = (width / 2.0) - rh;
  // excess *=-1;
  double lv = rv + (height / 2.0);
  double lh = lv * tan(theta);
  // lh = abs(lh);
  double x = lh - excess;

  // printf("ro: %4f th: %4f rh: %4f rv: %4f ex: %4f lv: %4f lh: %4f x: %4f\n", rho, theta ,rh, rv, excess, lv,lh,x);

  return x;
}

bool parametricIntersect(float r1, float t1, float r2, float t2, int &x, int &y) {
  float ct1 = cosf(t1);     //matrix element a
  float st1 = sinf(t1);     //b
  float ct2 = cosf(t2);     //c
  float st2 = sinf(t2);     //d
  float d = ct1 * st2 - st1 * ct2;        //determinative (rearranged matrix for inverse)
  if (d != 0.0f) {
    x = (int) ((st2 * r1 - st1 * r2) / d);
    y = (int) ((-ct2 * r1 + ct1 * r2) / d);
    return (true);
  } else { //lines are parallel and will NEVER intersect!
    return (false);
  }
}

void draw_lines(Mat &image, const vector<Vec2f> lines, Scalar color) {
  for (size_t i = 0; i < lines.size(); i++) {

    draw_line(image, lines[i], color);
  }
}

void draw_line(Mat &image, Vec2f line, Scalar color) {
  float theta = line[0], rho = line[1];
  vector<Point> p = to_points(theta, rho);
  cv::line(image, p[0], p[1], color, 3, CV_AA);
}

vector<Point> to_points(float theta, float rho) {
  vector<Point> rv = {Point(), Point()};
  double a = cos(theta), b = sin(theta);
  double x0 = a * rho, y0 = b * rho;
  rv[0].x = cvRound(x0 + 1000 * (-b));
  rv[0].y = cvRound(y0 + 1000 * (a));
  rv[1].x = cvRound(x0 - 1000 * (-b));
  rv[1].y = cvRound(y0 - 1000 * (a));
  return rv;
}

Vec2f normalize_point(Vec2f point) {
  point[0] -= deg2rad(360);
  while (point[0] < 0) {
    point[0] += deg2rad(360);
  }
  return point;
}