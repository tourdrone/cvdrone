// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2013 puku0x
// https://github.com/puku0x/cvdrone
//
// This source file is part of CV Drone library.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of EITHER:
// (1) The GNU Lesser General Public License as published by the Free
//     Software Foundation; either version 2.1 of the License, or (at
//     your option) any later version. The text of the GNU Lesser
//     General Public License is included with this library in the
//     file cvdrone-license-LGPL.txt.
// (2) The BSD-style license that is included with this library in
//     the file cvdrone-license-BSD.txt.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
// cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initCommand()
// Description  : Initialize AT command.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::initCommand(void)
{
    // Open the IP address and port
    if (!sockCommand.open(ip, ARDRONE_COMMAND_PORT)) {
        ardError("UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_COMMAND_PORT, __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::takeoff()
// Description  : Take off the AR.Drone.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::takeoff(void)
{
    resetEmergency();
    sockCommand.sendf("AT*REF=%d,290718208\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::landing()
// Description  : Land the AR.Drone.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::landing(void)
{
    sockCommand.sendf("AT*REF=%d,290717696\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::emergency()
// Description  : Emergency stop.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::emergency(void)
{
    sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::move(X velocity[m/s], Y velocity[m/s], Rotational speed[rad/s])
// Description  : Move the AR.Drone in 2D plane.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::move(double vx, double vy, double vr)
{
    move3D(vx, vy, 0.0, vr);
}

// --------------------------------------------------------------------------
// ARDrone::move3D(X velocity[m/s], Y velocity[m/s], Z velocity[m/s], Rotational speed[rad/s])
// Description  : Move the AR.Drone in 3D space.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::move3D(double vx, double vy, double vz, double vr)
{
    const float gain = 0.4f;
    float v[4] = {-vy*gain, -vx*gain, vz*gain, -vr*gain};
    int mode = (fabs(vx) > 0.0 || fabs(vy) > 0.0);
    sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, mode, *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]));
}

// --------------------------------------------------------------------------
// ARDrone::setCamera(Camera channel)
// Description  : Change the camera channel.
//                AR.Drone1.0 supports 0, 1, 2, 3.
//                AR.Drone2.0 supports 0, 1.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::setCamera(int channel)
{
    // ARDrone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", seq++, channel % 2);
        Sleep(100);
    }
    // ARDrone 1.0
    else {
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", seq++, channel % 4);
        Sleep(100);
    }
}

// --------------------------------------------------------------------------
// ARDrone::setAnimation(Flight animation ID, Duration[s])
// Description  : Run specified flight animation.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::setAnimation(int id, int duration)
{
    sockCommand.sendf("AT*ANIM=%d,%d,%d\r", seq++, id, duration);
    //sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
    //sockCommand.sendf("AT*CONFIG=%d,\"leds:flight_anim\",\"%d,%d\"\r", seq++, id, duration);
    //Sleep(100);
}

// --------------------------------------------------------------------------
// ARDrone::setLED(LED animation ID, Frequency[Hz], Duration[s])
// Description  : Run specified LED animation.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::setLED(int id, float freq, int duration)
{
    sockCommand.sendf("AT*LED=%d,%d,%d,%d\r", seq++, id, *(int*)(&freq), duration);
    //sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
    //sockCommand.sendf("AT*CONFIG=%d,\"leds:leds_anim\",\"%d,%d,%d\"\r", seq++, id, *(int*)(&freq), duration);
    //Sleep(100);
}

// --------------------------------------------------------------------------
// ARDrone::startVideoRecord()
// Start recording video.
// This function is only for AR.Drone 2.0
// You should set a USB key with > 100MB to your drone
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::startVideoRecord(void)
{
    if (version.major == ARDRONE_VERSION_2) {
        // Finalize video
        finalizeVideo();

        // Enable video record
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"TRUE\"\r", seq++);
        Sleep(100);

        // Output video with MP4_360P_H264_720P_CODEC
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x82);
        Sleep(100);

        // Initialize video
        initVideo();
    }
}

// --------------------------------------------------------------------------
// ARDrone::stopVideoRecord()
// Stop recording video.
// This function is only for AR.Drone 2.0
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::stopVideoRecord(void)
{
    if (version.major == ARDRONE_VERSION_2) {
        // Finalize video
        finalizeVideo();

        // Enable video record
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"FALSE\"\r", seq++);
        Sleep(100);

        // Output video with 360P
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x81);
        Sleep(100);

        // Initialize video
        initVideo();
    }
}

// --------------------------------------------------------------------------
// ARDrone::resetWatchDog()
// Description  : Stop hovering.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::resetWatchDog(void)
{
    // If AR.Drone is in Watch-Dog, reset it
    if (navdata.ardrone_state & ARDRONE_COM_WATCHDOG_MASK) sockCommand.sendf("AT*COMWDG=%d\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::resetEmergency()
// Description  : Disable the emergency lock.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::resetEmergency(void)
{
    // If AR.Drone is in emergency, reset it
    if (navdata.ardrone_state & ARDRONE_EMERGENCY_MASK) sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::finalizeCommand()
// Description  : Finalize AT command
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeCommand(void)
{
    // Close the socket
    sockCommand.close();
}