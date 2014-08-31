#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Thresholds
    int minH = 0, maxH = 255;
    int minS = 0, maxS = 255;
    int minV = 0, maxV = 255;

    // XML save data
    std::string filename("thresholds.xml");
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    // If there is a save file then read it
    if (fs.isOpened()) {
        maxH = fs["H_MAX"];
        minH = fs["H_MIN"];
        maxS = fs["S_MAX"];
        minS = fs["S_MIN"];
        maxV = fs["V_MAX"];
        minV = fs["V_MIN"];
        fs.release();
    }

    // Create a window
    cv::namedWindow("binalized");
    cv::createTrackbar("H max", "binalized", &maxH, 255);
    cv::createTrackbar("H min", "binalized", &minH, 255);
    cv::createTrackbar("S max", "binalized", &maxS, 255);
    cv::createTrackbar("S min", "binalized", &minS, 255);
    cv::createTrackbar("V max", "binalized", &maxV, 255);
    cv::createTrackbar("V min", "binalized", &minV, 255);
    cv::resizeWindow("binalized", 0, 0);

    // Kalman filter
    cv::KalmanFilter kalman(4, 2, 0);

    // Sampling time [s]
    const double dt = 1.0;

    // Transition matrix (x, y, vx, vy)
    cv::Mat1f A(4, 4);
    A << 1.0, 0.0,  dt, 0.0,
         0.0, 1.0, 0.0,  dt,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    kalman.transitionMatrix = A;

    // Measurement matrix (x, y)
    cv::Mat1f H(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    kalman.measurementMatrix = H;

    // Process noise covairance (x, y, vx, vy)
    cv::Mat1f Q(4, 4);
    Q << 1e-5,  0.0,  0.0,  0.0,
          0.0, 1e-5,  0.0,  0.0,
          0.0,  0.0, 1e-5,  0.0,
          0.0,  0.0,  0.0, 1e-5;
    kalman.processNoiseCov = Q;

    // Measurement noise covariance (x, y)
    cv::Mat1f R(2, 2);
    R << 1e-1,  0.0,
          0.0, 1e-1;
    kalman.measurementNoiseCov = R;

	char textBuffer[80];
	cv::Scalar green = CV_RGB(0,255,0);
	float speed = 0.0;
	bool learnMode = false;

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // HSV image
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV_FULL);

        // Binalize
        cv::Mat binalized;
        cv::Scalar lower(minH, minS, minV);
        cv::Scalar upper(maxH, maxS, maxV);
        cv::inRange(hsv, lower, upper, binalized);

        // Show result
        cv::imshow("binalized", binalized);

        // De-noising
        cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binalized, binalized, cv::MORPH_CLOSE, kernel);
        //cv::imshow("morphologyEx", binalized);

        // Detect contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binalized.clone(), contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        // Find largest contour
        int contour_index = -1;
        double max_area = 0.0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = fabs(cv::contourArea(contours[i]));
            if (area > max_area) {
                contour_index = i;
                max_area = area;
            }
        }

        // Object detected
        if (contour_index >= 0) {
            // Moments
            cv::Moments moments = cv::moments(contours[contour_index], true);
            double marker_y = (int)(moments.m01 / moments.m00);
            double marker_x = (int)(moments.m10 / moments.m00);

            // Measurements
            cv::Mat measurement = (cv::Mat1f(2, 1) << marker_x, marker_y);

            // Correction
            cv::Mat estimated = kalman.correct(measurement);

            // Show result
            cv::Rect rect = cv::boundingRect(contours[contour_index]);
            cv::rectangle(image, rect, cv::Scalar(0, 255, 0));
        }

        // Prediction
        cv::Mat1f prediction = kalman.predict();
        int radius = 1e+3 * kalman.errorCovPre.at<float>(0, 0);

        // Show predicted position
        cv::circle(image, cv::Point(prediction(0, 0), prediction(0, 1)), radius, green, 2);

		// Calculate object heading fraction
		float heading = -((image.cols/2)-prediction(0, 0))/(image.cols/2);
		sprintf(textBuffer, "heading = %+3.2f", heading);
		putText(image, textBuffer, cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
		
		// Control drone
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 0x260000) vx =  1.0;
        if (key == 0x280000) vx = -1.0;
        if (key == 0x250000) vr =  1.0;
        if (key == 0x270000) vr = -1.0;
        if (key == 'q')      vz =  1.0;
        if (key == 'a')      vz = -1.0;
		if (key == 'l')      learnMode = !learnMode;
		if ((key >= '0') && (key <= '9')) 
		{
			speed = (key-'0')*0.1;
			//printf("speed = %3.2f\n", speed);
		}
		sprintf(textBuffer, "speed = %3.2f", speed);
		putText(image, textBuffer, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);

		if (key == -1)
		{//No key hit - chase the object
			vx=speed;
			vr = -heading*2;
		}
        ardrone.move3D(vx, vy, vz, vr);

		// Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) 
			{
				ardrone.takeoff();
			}
            else
			{
				ardrone.landing();
			}
        }
		

		if(learnMode) {
			cv::Vec3b bgr = image.at<cv::Vec3b>(cvPoint(image.cols/2, image.rows/2));

			// Crosshairs
			cv::line(image, cvPoint(image.cols/2, 0), cvPoint(image.cols/2, image.rows/2 - 2), green); //top vertical crosshair
			cv::line(image, cvPoint(image.cols/2, image.rows/2 + 2), cvPoint(image.cols/2, image.rows), green); //bottom vertical crosshair
			cv::line(image, cvPoint(0, image.rows/2), cvPoint(image.cols/2 - 2, image.rows/2), green); //left horizontal crosshair
			cv::line(image, cvPoint(image.cols/2 + 2, image.rows/2), cvPoint(image.cols, image.rows/2), green); //right horizontal crosshair

			sprintf(textBuffer, "sample = %3d, %3d, %3d", bgr[0], bgr[1], bgr[2]);
			putText(image, textBuffer, cvPoint(30,90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, green, 1, CV_AA);
		}

        // Display the image
        cv::imshow("camera", image);
    }

    // Save thresholds
    fs.open(filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        cv::write(fs, "H_MAX", maxH);
        cv::write(fs, "H_MIN", minH);
        cv::write(fs, "S_MAX", maxS);
        cv::write(fs, "S_MIN", minS);
        cv::write(fs, "V_MAX", maxV);
        cv::write(fs, "V_MIN", minV);
        fs.release();
    }

    // See you
    ardrone.close();

    return 0;
}