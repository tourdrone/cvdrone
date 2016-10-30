#ifndef OBJ_FOLLOWING_HEADER
#define OBJ_FOLLOWING_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

class ObjectFollowing {
  public:
    int minH = 0;
    int maxH = 255;
    int minS = 0;
    int maxS = 255;
    int minV = 0;
    int maxV = 255;

    //int targetH;
    //int targetS
    //int targetV
    cv::KalmanFilter kalman;
    bool learnMode = false;
    bool moveStatus = false;
    cv::Rect rect;
    int rect_area = 0;

    void initialize();
    void close();
    ControlMovements fly(cv::Mat *image, int key);
    ControlMovements detectObject(cv::Mat image, int key);
    void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value);
};

/*
*/
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance);

#endif
