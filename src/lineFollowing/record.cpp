//
// Created by Caleb on 4/13/17.
//

#include "record.h"
#include "../control.h"

using namespace std;
using namespace cv;

Record::Record(Control *control) {
//  file = FileStorage("some_name.ext", cv::FileStorage::WRITE);
//  c = cv::VideoCapture();
  control_ptr = control;
//  cv::VideoWriter output_cap("some_file",
//                             control_ptr->image.get(CV_CAP_PROP_FOURCC),
//                             input_cap.get(CV_CAP_PROP_FPS),
//                             cv::Size(input_cap.get(CV_CAP_PROP_FRAME_WIDTH),
//                                      input_cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
//  printf("before thing\n");
  v = cv::VideoWriter("caleb_file.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25,
                      Size(640, 360), true);
//  printf("writing %dx%d\n", control_ptr->image.cols, control_ptr->image.rows);
//  printf("after thing\n");
//  printf("%d\n", (bool) v.isOpened());
}

void Record::run() {
//  printf("writing %dx%d\n", control_ptr->image.cols, control_ptr->image.rows);
  if (control_ptr->image.empty())
    return;
  v.write(control_ptr->image);
//  imshow("test", control_ptr->image);
//  v << control_ptr->image;
}

void Record::close() {


}
