/*

*/
#ifndef LINE_FOLLOWING_HEADER
#define LINE_FOLLOWING_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

using namespace cv;
using namespace std;

class Control;

/*
*/
class LineFollowing {
public:
  LineFollowing(Control *control);

  void close();

  void fly();

private:
  Control *control_ptr;

  void detect_lines(cv::Mat &original_frame);

  double minH, minS, minV, maxH, maxS, maxV;
  double width, height;
  std::vector<cv::Vec2f> found_lines;

  double distance_from_center(float rho, float theta, double width, double height);

  cv::Point find_intersection(cv::Vec2f a, cv::Vec2f b);
};

void compress_lines(vector<Vec2f> &condensed, const vector<Vec2f> &tmp_list);

void draw_lines(Mat &image, const vector<Vec2f> &lines);

vector<Vec2f> condense_lines(vector<Vec2f> lines, bool keep_going);

vector<Point> to_points(float theta, float rho);

bool parametricIntersect(float r1, float t1, float r2, float t2, int &x, int &y);

Vec2f normalize_point(Vec2f point);


#endif
