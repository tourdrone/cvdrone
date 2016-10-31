
/*
 */
#include "ardrone/ardrone.h"
#include "structures.h"
#include <string>
#include <chrono>
#include <thread>
#include <ctime>

using namespace std;

/*
 */

class ManualFlying;
class LineFollowing;
class ObjectFollowing;

class Control {
  public:
    const string flightLog = "flight_log.txt";

    //AR.Drone class
    ARDrone ardrone;

    friend class ManualFlying;
    friend class ObjectFollowing;
    friend class LineFollowing;

    FILE *flight_log;

	Control();
    void close();
    void fly();
	bool getKey(int wait);
    void detectFlyingMode();
    void changeSpeed();
    void detectTakeoff();
    void getImage();
    void move();

    void overlayControl();

  private:

    cv::Mat image;
    cv::Scalar green; //overlay putText color value
    FlyingMode flyingMode = Manual;
    ControlMovements velocities;
    time_t takeoff_time;
    double speed = 0.0;
    int key;
    int batteryPercentage;
    int altitude;
    bool flying;

    ManualFlying *manualFlying;
    ObjectFollowing *objectFollowing;
    LineFollowing *lineFollowing;
};
