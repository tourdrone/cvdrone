/*

*/

#include "lineFollowing.h"
#include "../control.h"

using namespace std;
using namespace cv;


void compress_lines(vector<Vec2f> &condensed, const vector<Vec2f> &tmp_list);

void draw_lines(Mat &image, const vector<Vec2f> &lines);

vector<Vec2f> condense_lines(vector<Vec2f> lines);

double deg2rad(double deg);

vector<Point> to_points(float theta, float rho);

bool parametricIntersect(float r1, float t1, float r2, float t2, int &x, int &y);

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

  vector<Vec2f> tmp = condense_lines(lines);
  sort(tmp.begin(), tmp.end(),
       [](const Vec2f &a, const Vec2f &b) {
         return a[0] < b[0];
       });
  if (tmp.size() > 0) {
    found_lines = tmp;
  }

  draw_lines(original_frame, found_lines);


}

vector<Vec2f> condense_lines(vector<Vec2f> lines) {
  vector<Vec2f> condensed;
  vector<Vec2f> tmp_list;
  for (int i = 0; i < (int) lines.size(); i++) {
//    if (lines[i][1] > CV_PI / 2) {
//      lines[i][1] -= CV_PI;
//      lines[i][0] *= -1;
//    }
//    else if (lines[i][1] < ((CV_PI / 2) * -1)) {
//      lines[i][1] += CV_PI;
//      lines[i][0] *= -1;
//    }
    lines[i][1] -= deg2rad(360);
    while (lines[i][1] < 0) {
      lines[i][1] += deg2rad(360);
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
      if (abs(to_manipulate[0] - tmp_list.front()[0]) < deg2rad(20)) {
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

double deg2rad(double deg) {
  return deg * (CV_PI / 180.0);
}

void draw_lines(Mat &image, const vector<Vec2f> &lines) {
  for (size_t i = 0; i < lines.size(); i++) {
    float theta = lines[i][0], rho = lines[i][1];
    // float rho = lines[i][0], theta = lines[i][1];Point pt1;
    vector<Point> p = to_points(theta, rho);
    line(image, p[0], p[1], Scalar(0, 0, 255), 3, CV_AA);
  }
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

void compress_lines(vector<Vec2f> &condensed, const vector<Vec2f> &tmp_list) {
  Vec2f new_point;
  float angle_sum = 0;
  float rho_sum = 0;
  for (int j = 0; j < (int) tmp_list.size(); ++j) {
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
  line(control_ptr->image, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);

  if (found_lines.size() > 1) {
    // I need to turn
    //TODO: move until line is centered
    //TODO: look for symbol, and turn
    origin_x = 0;
    origin_y = 0;
    Vec2i point = find_intersection(found_lines[0], found_lines[1]);
    printf("My coords are x: %3d y %3d\n", point[0], point[1]);
    line(control_ptr->image, cvPoint(origin_x + point[0] + 10, origin_y + point[1]),
         cvPoint(origin_x + point[0] - 10, origin_y + point[1]), Scalar(0, 255, 0), 3, CV_AA);
    line(control_ptr->image, cvPoint(origin_x + point[0], origin_y + point[1] + 10),
         cvPoint(origin_x + point[0], origin_y + point[1] - 10), Scalar(0, 255, 0), 3, CV_AA);
    return;
  }

  else {
    // I need to snap myself to the line
    if (found_lines[0][0] >= deg2rad(5)) {
      control_ptr->velocities.vr = -.2;
      printf("Turning Right\n");
    }
    else if (found_lines[0][0] <= deg2rad(-1 * 5)) {
      control_ptr->velocities.vr = .2;
      printf("Turning Left\n");
    } else {
      printf("Checking Distance\n");

      double offset = calculated_distance;
      // printf("Offset is: %5.2f with a distance of %5.2f and width of %5.2f halved to %5.2f\n", offset,
      // found_lines[0][1],
      // (double) control_ptr->image.cols, (control_ptr->image.cols / 2.0));
      if (-100 <= offset && offset <= 100) {
        printf("No need to move\n");
      } else if (offset < 0) {
        //we are to the right of the line
        //we need to move left
        control_ptr->velocities.vy = 1;
        printf("Move left\n");
      } else {
        //we need to move right
        control_ptr->velocities.vy = -1;
        printf("Move right\n");
      }
    }
  }


  return;
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

cv::Point LineFollowing::find_intersection(Vec2f a, Vec2f b) {
  Point rv;
  bool do_intersect = parametricIntersect(a[1], a[0], b[1], b[0], rv.x, rv.y);
  if (!do_intersect) {
    printf("No intersection!\n");
  }

  return rv;
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
