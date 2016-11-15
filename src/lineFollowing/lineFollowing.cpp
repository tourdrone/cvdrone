/*

*/

#include "lineFollowing.h"
#include "line_utilities.h"
#include "../control.h"
#include "line_simplification.h"

using namespace std;
using namespace cv;


void LineFollowing::detect_lines(Mat &original_frame) {

  Mat hsv;
  Mat mask;
  width = original_frame.cols;
  height = original_frame.rows;

  cvtColor(original_frame, hsv, CV_BGR2HSV); // Image is now HSV

  Scalar lower(minH, minS, minV);
  Scalar upper(maxH, maxS, maxV);
  inRange(hsv, lower, upper, mask); // Create a mask of only the desired color
  Canny(mask, mask, 50, 200, 3);

  vector<Vec2f> lines;
  HoughLines(mask, lines, 1, CV_PI / 180, 100, 0, 0);

  // printf("Adding in %d lines\n", (int) lines.size());

  vector<Vec2f> tmp = condense_lines(lines, true);
  sort(tmp.begin(), tmp.end(),
       [](const Vec2f &a, const Vec2f &b) {
         return a[0] < b[0];
       });
  if (tmp.size() > 0) {
    found_lines = tmp;
  }

  draw_lines(original_frame, found_lines);

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
  // cout << "line following\n";
  detect_lines(control_ptr->image);


  control_ptr->velocities.vx = 0;
  control_ptr->velocities.vy = 0;
  control_ptr->velocities.vz = 0;
  control_ptr->velocities.vr = 0;

  if (found_lines.size() < 1) {
    //I found no lines
    // printf("Found nothing\n");
    return;
  }
  double calculated_distance = distance_from_center(found_lines[0][1], found_lines[0][0], control_ptr->image.cols,
                                                    control_ptr->image.rows);

  // printf("calc dist of %f ", calculated_distance);
  int origin_x = 0 + (control_ptr->image.cols / 2);
  int origin_y = 0 + (control_ptr->image.rows / 2);
  Point pt1 = cvPoint(origin_x, origin_y);
  Point pt2 = cvPoint(origin_x + cvRound(calculated_distance), origin_y);
  line(control_ptr->image, pt1, pt2, Scalar(255, 255, 255), 3, CV_AA);

  if (found_lines.size() > 1) {
    // I need to turn
    //TODO: move until line is centered
    //TODO: look for symbol, and turn
    origin_x = 0;
    origin_y = 0;
    Vec2i point = find_intersection(found_lines[0], found_lines[1]);

    line(control_ptr->image, cvPoint(origin_x + point[0] + 10, origin_y + point[1]),
         cvPoint(origin_x + point[0] - 10, origin_y + point[1]), Scalar(255, 255, 255), 3, CV_AA);
    line(control_ptr->image, cvPoint(origin_x + point[0], origin_y + point[1] + 10),
         cvPoint(origin_x + point[0], origin_y + point[1] - 10), Scalar(255, 255, 255), 3, CV_AA);
    if (found_lines.size() >= 3) {
      for (int j = 0; j < (int) found_lines.size(); j++) {
        printf("(%5.1f, %5.0f) ", rad2deg(found_lines[j][0]) + 180, found_lines[j][1]);
      }
      printf("\n");
    }
    return;
  }

  else {
    // I need to snap myself to the line
    if (found_lines[0][0] >= deg2rad(5)) {
      control_ptr->velocities.vr = -.2;
      // printf("Turning Right\n");
    }
    else if (found_lines[0][0] <= deg2rad(-1 * 5)) {
      control_ptr->velocities.vr = .2;
      // printf("Turning Left\n");
    } else {
      // printf("Checking Distance\n");

      double offset = calculated_distance;
      // printf("Offset is: %5.2f with a distance of %5.2f and width of %5.2f halved to %5.2f\n", offset,
      // found_lines[0][1],
      // (double) control_ptr->image.cols, (control_ptr->image.cols / 2.0));
      if (-100 <= offset && offset <= 100) {
        // printf("No need to move\n");
      } else if (offset < 0) {
        //we are to the right of the line
        //we need to move left
        control_ptr->velocities.vy = 1;
        // printf("Move left\n");
      } else {
        //we need to move right
        control_ptr->velocities.vy = -1;
        // printf("Move right\n");
      }
    }
  }


  return;
}

cv::Point LineFollowing::find_intersection(Vec2f a, Vec2f b) {
  Point rv;
  bool do_intersect = parametricIntersect(a[1], a[0], b[1], b[0], rv.x, rv.y);
  if (!do_intersect) {
    printf("No intersection!\n");
  }

  return rv;
}

