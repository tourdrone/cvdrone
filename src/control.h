/*
*/
#include "ardrone/ardrone.h"
#include <string>
#include "structures.h"
#include "objectFollowing/objectFollowing.h"
#include "manual/manual.h"
#include "lineFollowing/lineFollowing.h"

using namespace std;

/*
*/
class Control {
  public:
    const string flightLog = "flight_log.txt";

    //AR.Drone class
    ARDrone ardrone;

    cv::Mat image;
    
    int key;
    FlyingMode flyingMode = Manual;
    double speed = 0.0;
    int batteryPercentage;
    bool flying;
    cv::Scalar green; //overlay putText color value

    ControlMovements velocities;

    FILE *flight_log;

    void initializeDroneControl(ObjectFollowing *objectFollowing);
    void detectFlyingMode();
    bool detectEscape();
    void changeSpeed();
    void detectTakeoff();
    void getImage();
    void move();

    void overlayControl();

    void close();
};
