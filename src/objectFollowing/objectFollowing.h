
#include "../ardrone/ardrone.h"

/*
*/
void initializeObjectFollowing(cv::KalmanFilter *kalman, int *maxHue, int *minHue, int *maxSaturation, int *minSaturation, int *maxValue, int *minValue);

/*
*/
void closeObjectFollowing(int maxHue, int minHue, int maxSaturation, int minSaturation, int maxValue, int minValue);

/*
*/
float detectObject(cv::Mat image, cv::KalmanFilter kalman, int minH, int maxH, int minS, int maxS, int minV, int maxV, bool learnMode, bool moveStatus, cv::Rect *rect);

/*
*/
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance);

/*
*/
void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value, bool moveStatus);
