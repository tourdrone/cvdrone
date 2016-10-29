
#include "../ardrone/ardrone.h"

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

    /*
    */
    void initializeObjectFollowing();

    /*
    */
    void closeObjectFollowing();

    /*
    */
    float detectObject(cv::Mat image, bool learnMode, bool moveStatus, cv::Rect *rect);
};


/*
*/
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance);

/*
*/
void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value, bool moveStatus);
