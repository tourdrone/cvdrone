#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        printf("Failed to initialize.\n");
        return -1;
    }

    // Map
    IplImage *map = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
    cvZero(map);

    // Position matrix
    cv::Mat P = cv::Mat::zeros(3, 1, CV_64FC1);

    // Main loop
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Orientation
        double roll  = ardrone.getRoll();
        double pitch = ardrone.getPitch();
        double yaw   = ardrone.getYaw();

        // Velocity
        double vx, vy, vz;
        double velocity = ardrone.getVelocity(&vx, &vy, &vz);

        // Rotation matrices
        double _RX[] = {        1.0,       0.0,        0.0,
                                0.0, cos(roll), -sin(roll),
                                0.0, sin(roll),  cos(roll)};
        double _RY[] = { cos(pitch),       0.0,  sin(pitch),
                                0.0,       1.0,        0.0,
                        -sin(pitch),       0.0,  cos(pitch)};
        double _RZ[] = {   cos(yaw), -sin(yaw),        0.0,
                           sin(yaw),  cos(yaw),        0.0,
                                0.0,       0.0,        1.0};
        cv::Mat RX(3, 3, CV_64FC1, _RX);
        cv::Mat RY(3, 3, CV_64FC1, _RY);
        cv::Mat RZ(3, 3, CV_64FC1, _RZ);

        // Time
        static double last = ardGetTickCount();
        double dt = (ardGetTickCount() - last) * 0.001;
        last = ardGetTickCount();

        // Local movement
        double _M[] = {vx * dt, vy * dt, vz * dt};
        cv::Mat M(3, 1, CV_64FC1, _M);

        // Dead reckoning
        P = P + RZ * RY * RX * M;

        // Position (x, y, z)
        double pos[3] = {P.at<double>(0,0), P.at<double>(1,0), P.at<double>(2,0)};

        // Take off / Landing
        if (KEY_PUSH(VK_SPACE)) {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
        if (KEY_DOWN(VK_UP))    x =  0.5;
        if (KEY_DOWN(VK_DOWN))  x = -0.5;
        if (KEY_DOWN(VK_LEFT))  r =  1.0;
        if (KEY_DOWN(VK_RIGHT)) r = -1.0;
        if (KEY_DOWN('Q'))      z =  0.5;
        if (KEY_DOWN('A'))      z = -0.5;
        ardrone.move3D(x, y, z, r);

        // Change camera
        static int mode = 0;
        if (KEY_PUSH('C')) ardrone.setCamera(++mode%4);

        // Display the image
        cvDrawCircle(map, cvPoint(-pos[1]*30.0 + map->width/2, -pos[0]*30.0 + map->height/2), 2, CV_RGB(255,0,0));
        cvShowImage("map", map);
        cvDrawText(image, cvPoint(20, 20), "x = %3.2f, y = %3.2f, z = %3.2f", pos[0], pos[1], pos[2]);
        cvShowImage("camera", image);
        cvWaitKey(1);
    }

    // See you
    ardrone.close();
    cvReleaseImage(&map);

    return 0;
}