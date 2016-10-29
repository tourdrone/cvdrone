
#include "../ardrone/ardrone.h"

/*
*/
void initializeObjectFollowing(int *maxHue, int *minHue, int *maxSaturation, int *minSaturation, int *maxValue, int *minValue);

/*
*/
void closeObjectFollowing(int maxHue, int minHue, int maxSaturation, int minSaturation, int maxValue, int minValue);

/*
*/
void setHSVTrackBarPositions(int hue, int saturation, int value, int tolerance);

/*
*/
void displayObjectFollowingInfo(cv::Mat *image, double heading, int hue, int saturation, int value);
