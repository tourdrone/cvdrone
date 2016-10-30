
/*
*/
#include "ardrone/ardrone.h"
#include "structures.h"
#include "objectFollowing/objectFollowing.h"
#include "manual/manual.h"
#include "lineFollowing/lineFollowing.h"
#include <string>
#include <chrono>
#include <thread>

using namespace std;

/*
*/
class Control {
  public:
    const string flightLog = "flight_log.txt";

    //AR.Drone class
    ARDrone ardrone;

    ManualFlying manualFlying;
    ObjectFollowing objectFollowing;
    LineFollowing lineFollowing;

    cv::Mat image;
    
    int key;
    FlyingMode flyingMode = Manual;
    double speed = 0.0;
    int batteryPercentage;
    int altitude;
    bool flying;
    cv::Scalar green; //overlay putText color value

    ControlMovements velocities;

    FILE *flight_log;

    void initialize();
    void close();
    ControlMovements fly();
    void detectFlyingMode();
    bool detectEscape();
    void changeSpeed();
    void detectTakeoff();
    void getImage();
    void move();

    void overlayControl();

};

