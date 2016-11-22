//
// Created by Caleb on 11/15/16.
//

#include "lineFollowing.h"
#include "line_simplification.h"
#include "line_utilities.h"

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

vector<Vec2f> condense_lines(vector<Vec2f> lines, bool keep_going) {
  bool debug = false;
  vector<Vec2f> condensed;
  vector<Vec2f> tmp_list;
  double diff;
  bool flipped;

  if (debug) {
    printf("\n================== Before Anything ===================\n");
    for (int i = 0; i < lines.size(); ++i) {
      printf("(%5.1f, %5.1f) ", rad2deg(lines[i][keep_going ? 1 : 0]), lines[i][keep_going ? 0 : 1]);
    }
    printf("\n");
  }

  if (keep_going) {
    for (int i = 0; i < (int) lines.size(); i++) {
      //put in order of theta, rho
      swap(lines[i][0], lines[i][1]);

      if (debug) {
        printf("Took line from (%5.1f, %5.1f) to ", rad2deg(lines[i][0]), lines[i][1]);

        //      lines[i] = normalize_point(lines[i]);
        printf("normalized (%5.1f, %5.1f) to ", rad2deg(lines[i][0]), lines[i][1]);
        flipped = false;
      }
      if (lines[i][0] >= deg2rad(90)) {
        if (debug) {
          flipped = true;
        }
        lines[i] = flip_line(lines[i]);
      }
      if (debug) {
        printf("oriented (%5.1f, %5.1f) ", rad2deg(lines[i][0]), lines[i][1]);
        if (flipped) { printf("And I flipped"); }
        printf("\n");
      }
    }
    // return lines;
  }

  if (debug) {
    printf("\n================== Before Sorting ===================\n");
    for (int i = 0; i < lines.size(); ++i) {
      printf("(%5.1f, %5.1f) ", rad2deg(lines[i][0]), lines[i][1]);
    }
    printf("\n");
  }

  //Order from least to greatest theta
  sort(lines.begin(), lines.end(),
       [](const Vec2f &a, const Vec2f &b) {
         return a[0] < b[0];
       });

  if (debug) {
    printf("\n================== After Pre-Processing ===================\n");
    for (int i = 0; i < lines.size(); ++i) {
      printf("(%5.1f, %5.1f) ", rad2deg(lines[i][0]), lines[i][1]);
    }
    printf("\n");
  }

  while (!lines.empty()) {
    Vec2f to_manipulate = lines.front();
    lines.erase(lines.begin());

    if (tmp_list.empty()) {
      tmp_list.push_back(to_manipulate);
      continue;
    } else {
      diff = abs(to_manipulate[0] - tmp_list.front()[0]);
      if (diff > deg2rad(180)) {
        diff = deg2rad(360) - diff;
      }
      if (diff < deg2rad(degree_tolerance)) {
        //The angles are similar
        if (abs(to_manipulate[1] - tmp_list.front()[1]) < distance_tolerance) {
          //the distances are similar
          tmp_list.push_back(to_manipulate);
          continue;
        } else {
          if (debug) {
            printf("The angles %6.1f and %6.1f are close enough\n", rad2deg(to_manipulate[0]), rad2deg(tmp_list.front()[0]));
            printf("distance of %4.1f and %4.1f is too much\n", to_manipulate[1], tmp_list.front()[1]);
          }
        }
      } else {
        if (debug) {
          printf("The angles %6.1f and %6.1f are too much\n", rad2deg(to_manipulate[0]), rad2deg(tmp_list.front()[0]));
        }
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
  if (condensed.size() >= 2) {
    condensed.back() = flip_line(condensed.back());

    if (keep_going) {
      if (debug) {
        printf("Second loop\n");
      }
      condensed = condense_lines(condensed, false);
    }
  }
  return condensed;
}