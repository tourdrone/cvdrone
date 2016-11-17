//
// Created by Caleb on 11/15/16.
//

#ifndef CVDRONE_LINE_SIMPLIFICATION_H
#define CVDRONE_LINE_SIMPLIFICATION_H

using namespace std;
static const int degree_tolerance = 10;

static const int distance_tolerance = 15;

void compress_lines(vector<cv::Vec2f> &condensed, const vector<cv::Vec2f> &tmp_list);

vector<Vec2f> condense_lines(vector<Vec2f> lines, bool keep_going);

#include "../ardrone/ardrone.h"

#endif //CVDRONE_LINE_SIMPLIFICATION_H
