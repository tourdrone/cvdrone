//
// Created by Caleb on 4/13/17.
//

#ifndef CVDRONE_RECORD_H
#define CVDRONE_RECORD_H

#include "../ardrone/ardrone.h"
#include "../structures.h"

using namespace cv;
using namespace std;

class Control;

class Record {
public:
  Record(Control *control);

  void run();

  void close();

private:
  Control *control_ptr;
  FileStorage file;
  cv::VideoWriter v;
  cv::VideoCapture c;
};


#endif //CVDRONE_RECORD_H
