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

vector<Vec2f> condense_lines(vector<Vec2f> lines) {
  bool debug = 0;
  vector<Vec2f> condensed;
  vector<Vec2f> tmp_list;
  double diff;
  bool flipped;

  if (debug) {
    printf("\n================== Before Anything ===================\n");
    for (int i = 0; i < (int) lines.size(); ++i) {
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
    printf("\n================== After Sorting ===================\n");
    for (int i = 0; i < (int) lines.size(); ++i) {
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

  if (debug) {
    printf("=================== Before End Lines Comparison =================\n");
    for (int j = 0; j < (int) condensed.size(); j++) {
      printf(" (%.2f, %.2f)", rad2deg(condensed[j][0]), condensed[j][1]);
    }
    printf("\n");
  }

  if (condensed.size() >= 2) {
    Vec2f *front = &condensed.front();
    Vec2f *back = &condensed.back();
    if ( (abs( ((*front)[0] - 0) - ((*back)[0] - CV_PI) ) < deg2rad(degree_tolerance)) && // thetas are close enough
            (( abs((*front)[1]) - abs((*back)[1]) ) < distance_tolerance) ) { // rhos are close enough
      // lines should be combined
      *back = flip_line(*back);
      (*front)[0] = ( (*front)[0] + (*back)[0] ) / 2.0F; // average thetas
      (*front)[1] = ( (*front)[1] + (*back)[1] ) / 2.0F; // average rhos
      condensed.pop_back(); // delete back line that was combined
    }
  }


  if(debug) {
    printf("=================== Lines Before Normalization =================\n");
    for (int j = 0; j < (int) condensed.size(); j++) {
      printf(" (%.2f, %.2f)", rad2deg(condensed[j][0]), condensed[j][1]);
    }
    printf("\n");
  }



  for (int i = 0; i < (int) condensed.size(); i++) {
    //put in order of theta, rho

    if (debug) {
      printf("Took line from (%5.1f, %5.1f) to ", rad2deg(condensed[i][0]), condensed[i][1]);
      flipped = false;
    }

    if (condensed[i][0] >= deg2rad(90)) {
      if (debug) {
        flipped = true;
      }
      condensed[i] = flip_line(condensed[i]);
    }

    if (debug) {
      printf("oriented (%5.1f, %5.1f) ", rad2deg(condensed[i][0]), condensed[i][1]);
      if (flipped) { printf("And I flipped"); }
      printf("\n");
    }

  }


  if(debug) {
    printf("=================== Final Lines =================\n");
    for (int j = 0; j < (int) condensed.size(); j++) {
      printf(" (%.2f, %.2f)", rad2deg(condensed[j][0]), condensed[j][1]);
    }
    printf("\n");
  }

  return condensed;
}