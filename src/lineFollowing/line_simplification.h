//
// Created by Caleb on 11/15/16.
//

#ifndef CVDRONE_LINE_SIMPLIFICATION_H
#define CVDRONE_LINE_SIMPLIFICATION_H

void compress_lines(vector<cv::Vec2f> &condensed, const vector<cv::Vec2f> &tmp_list);

vector<Vec2f> condense_lines(vector<Vec2f> lines, bool keep_going);

#include "../ardrone/ardrone.h"

#endif //CVDRONE_LINE_SIMPLIFICATION_H
