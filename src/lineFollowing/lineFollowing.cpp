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

  imshow("line_window", mask);

  Canny(mask, mask, 50, 200, 3);


  vector<Vec2f> lines;
  HoughLines(mask, lines, 1, CV_PI / 180, 100, 0, 0);

  vector<Vec2f> tmp = condense_lines(lines, true);

  sort(tmp.begin(), tmp.end(),
       [](const Vec2f &a, const Vec2f &b) {
         return a[0] < b[0];
       });
  if (tmp.size() > 0) {
    found_lines = tmp;
  }

  draw_lines(original_frame, found_lines, Scalar(255, 255, 255));

}

LineFollowing::LineFollowing(Control *control) {
  namedWindow("line_window", CV_WINDOW_NORMAL);

  control_ptr = control;

  maxH = 166;
  minH = 66;
  maxS = 143;
  minS = 43;
  maxV = 254;
  minV = 154;

  kp = .0018;
  ki = .0;
  kd = 0.0;

  time = 0;
  myfile.open("csv/output.csv");


  vertical_pid = new PID(1.0 / 25, 1, -1, kp, kd, ki);
  horizontal_pid = new PID(1.0 / 25, 1, -1, kp, kd, ki);

  return;
}

void LineFollowing::close() {
  return;
}

void LineFollowing::fly() {
  int center_x = 0 + (control_ptr->image.cols / 2);
  int center_y = 0 + (control_ptr->image.rows / 2);
  Point center_point = cvPoint(center_x, center_y);
  int tolerance = 10;
  line_options categorization;
  Point intersection_point;
  double calculated_distance_from_vertical = 0, calculated_distance_from_horizontal = 0;

  control_ptr->velocities.vx = 0;
  control_ptr->velocities.vy = 0;
  control_ptr->velocities.vz = 0;
  control_ptr->velocities.vr = 0;

  detect_lines(control_ptr->image);


  if (found_lines.size() < 1) return;

  // If there is no corner
  if (found_lines.size() < 3) {

    if (found_lines.size() == 1) {
      // Find "intersection_point" that drone should go towards
      // intersection_point is point where single line intersects a  horizontal line across the vertical middle of the camera feed
      categorization.vertical = found_lines[0];
      categorization.horizontal = Vec2f();
      calculated_distance_from_vertical = distance_from_center(categorization.vertical[1], categorization.vertical[0],
                                                               control_ptr->image.cols, control_ptr->image.rows);
      calculated_distance_from_horizontal = 0;
      intersection_point = cvPoint(center_x + cvRound(calculated_distance_from_vertical), center_y);

    } else if (found_lines.size() == 2) {// intersection_point is the intersection of the two lines

      if (abs(found_lines[0][0]) < abs(found_lines[1][0])) {
        categorization.vertical = found_lines[0];
        categorization.horizontal = found_lines[1];
      } else {
        categorization.vertical = found_lines[1];
        categorization.horizontal = found_lines[0];
      }
      printf("(%6.1f, %6.1f) (%6.1f, %6.1f)\n", rad2deg(categorization.vertical[0]), categorization.vertical[1],
             rad2deg(categorization.horizontal[0]), categorization.horizontal[1]);

      calculated_distance_from_vertical = distance_from_center(categorization.vertical[1], categorization.vertical[0],
                                                               control_ptr->image.cols, control_ptr->image.rows);
      calculated_distance_from_horizontal = distance_from_center(categorization.horizontal[1],
                                                                 categorization.horizontal[0],
                                                                 control_ptr->image.cols, control_ptr->image.rows);

      intersection_point = find_intersection(categorization.vertical, categorization.horizontal);

    }

    // Draw line to intersection_point
    line(control_ptr->image, center_point, intersection_point, color_white, 3, CV_AA);

    // Draw a cross at the intersection
    line(control_ptr->image, cvPoint(0 + intersection_point.x + 10, 0 + intersection_point.y),
         cvPoint(0 + intersection_point.x - 10, 0 + intersection_point.y), color_white, 3, CV_AA);
    line(control_ptr->image, cvPoint(0 + intersection_point.x, 0 + intersection_point.y + 10),
         cvPoint(0 + intersection_point.x, 0 + intersection_point.y - 10), color_white, 3, CV_AA);

    // Rotate to make line vertical
    if (categorization.vertical[0] >= deg2rad(5)) {
//      printf("Angle is %5.1f\n", rad2deg(categorization.vertical[0]));
      control_ptr->velocities.vr = -.1;
    } else if (categorization.vertical[0] <= deg2rad(-1 * 5)) {
//      printf("Angle is %5.1f\n", rad2deg(categorization.vertical[0]));
      control_ptr->velocities.vr = .1;
    }


    control_ptr->velocities.vy = 1 * vertical_pid->calculate(0, calculated_distance_from_vertical);
    control_ptr->velocities.vx = 1 * horizontal_pid->calculate(0, calculated_distance_from_horizontal);

    myfile << time++ << ", " << calculated_distance_from_horizontal << ", " << control_ptr->velocities.vx << endl;
//    printf("%f\n", control_ptr->velocities.vx);

//      return;

    /*// Move vertically
    int vertical_tolerance = 50;
    int horizontal_tolerance = 100;
    if (calculated_distance_from_vertical <= -1 * vertical_tolerance ||
        calculated_distance_from_vertical >= vertical_tolerance) {
      if (calculated_distance_from_vertical < 0) {
        //we are to the right of the intersection_point
        //we need to move left
        control_ptr->velocities.vy = 1;
      } else {
        //we need to move right
        control_ptr->velocities.vy = -1;
      }
    }
    // Move horizontally
    if (calculated_distance_from_horizontal <= -1 * horizontal_tolerance ||
        calculated_distance_from_horizontal >= horizontal_tolerance) {
      //todo move up or down the line

      if (calculated_distance_from_horizontal > 0) {
        //          we are above the intersection_point
        //          so we need to move backwards
        //          I think
        //          todo check this
        control_ptr->velocities.vx = -1;

      } else {
        control_ptr->velocities.vx = 1;
      }
    }*/


  } else if (found_lines.size() == 3) {

    // Categorize lines
    for (int i = 0; i < found_lines.size(); ++i) {
      float angle = abs(found_lines[i][0]);

      if (angle > deg2rad(0 - tolerance) && angle < deg2rad(0 + tolerance) && categorization.vertical == Vec2f()) {
        categorization.vertical = found_lines[i];
      }
      if (angle > deg2rad(90 - tolerance) && angle < deg2rad(90 + tolerance) && categorization.horizontal == Vec2f()) {
        categorization.horizontal = found_lines[i];
      }
      if (angle > deg2rad(45 - tolerance) && angle < deg2rad(45 + tolerance) && categorization.sloped == Vec2f()) {
        categorization.sloped = found_lines[i];
        if (found_lines[i][0] < 0) {
          categorization.d = direction::right;
        } else if (found_lines[i][0] > 0) {
          categorization.d = direction::left;
        }
      }

    }
  } else {
    //print out the points of all 4+ lines
    printf("Found more than 3 lines: ");
    for (int j = 0; j < (int) found_lines.size(); j++) {
      printf("(%5.1f, %5.0f) ", rad2deg(found_lines[j][0]) + 180, found_lines[j][1]);
    }
    printf("\n");
  }

  //Draw all the lines
  if (categorization.horizontal != Vec2f()) {
    draw_line(control_ptr->image, categorization.horizontal, color_green);
  }
  if (categorization.vertical != Vec2f()) {
    draw_line(control_ptr->image, categorization.vertical, color_blue);
  }
  if (categorization.sloped != Vec2f()) {
    draw_line(control_ptr->image, categorization.sloped, color_red);
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

