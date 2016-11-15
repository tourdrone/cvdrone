//
// Created by Caleb on 11/15/16.
//

#ifndef CVDRONE_LINE_UTILITIES_H
#define CVDRONE_LINE_UTILITIES_H

#include "../ardrone/ardrone.h"

cv::Vec2f flip_line(cv::Vec2f line);

double deg2rad(double deg);

double rad2deg(double rad);

bool parametricIntersect(float r1, float t1, float r2, float t2, int &x, int &y);

void draw_lines(Mat &image, const vector<Vec2f> &lines);

vector<Point> to_points(float theta, float rho);

Vec2f normalize_point(Vec2f point);

#endif //CVDRONE_LINE_UTILITIES_H
