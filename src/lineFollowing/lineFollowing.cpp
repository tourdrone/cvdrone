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

  draw_lines(original_frame, found_lines, Scalar(255, 255, 255));

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


    // Find "intersection_point" that drone should go towards

    if (found_lines.size() == 1) {
      // intersection_point is point where single line intersects a  horizontal line across the vertical middle of the camera feed
      categorization.vertical = found_lines[0];
      calculated_distance_from_vertical = distance_from_center(categorization.vertical[1], categorization.vertical[0],
          control_ptr->image.cols, control_ptr->image.rows);
      calculated_distance_from_horizontal = 0;
      intersection_point = cvPoint(center_x + cvRound(calculated_distance_from_vertical), center_y);
    } else if (found_lines.size() == 2) {
      // intersection_point is the intersection of the two lines

      //TODO maybe this is a very bad assumption to make, that [0] is the vertical line
      categorization.vertical = found_lines[0];
      categorization.horizontal = found_lines[1];
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
      control_ptr->velocities.vr = -.2;
      // printf("Turning Right\n");
    } else if (categorization.vertical[0] <= deg2rad(-1 * 5)) {
      control_ptr->velocities.vr = .2;
      // printf("Turning Left\n");
    } else {
      // printf("Checking Distance\n");
      // printf("Offset is: %5.2f with a distance of %5.2f and width of %5.2f halved to %5.2f\n", offset,

      // Move vertically
      // (double) control_ptr->image.cols, (control_ptr->image.cols / 2.0));
      if (-100 >= calculated_distance_from_vertical || calculated_distance_from_vertical >= 100) {
        if (calculated_distance_from_vertical < 0) {
          //we are to the right of the intersection_point
          //we need to move left
          control_ptr->velocities.vy = 1;
          // printf("Move left\n");
        } else {
          //we need to move right
          control_ptr->velocities.vy = -1;
          // printf("Move right\n");
        }
      }

      // Move horizontally
      if (-100 >= calculated_distance_from_horizontal || calculated_distance_from_horizontal >= 100) {
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
      }
    }
  } else if ( found_lines.size == 3 ){

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

