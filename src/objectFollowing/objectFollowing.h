#ifndef OBJ_FOLLOWING_HEADER
#define OBJ_FOLLOWING_HEADER

#include "../ardrone/ardrone.h"
#include "../structures.h"

class Control;

class ObjectFollowing {
  public:
    //int targetH;
    //int targetS
    //int targetV

	ObjectFollowing(Control *control);
    cv::KalmanFilter kalman;
    cv::Rect rect;

    void fly();
    void close();
	void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value);

  private:
  	Control *control_ptr;
    int minH = 0;
    int maxH = 255;
    int minS = 0;
    int maxS = 255;
    int minV = 0;
    int maxV = 255;
    int rect_area = 0;
    bool learnMode = false;
    bool moveStatus = false;
};

/*
*/
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance);

#endif
