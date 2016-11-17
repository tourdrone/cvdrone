# INTRODUCTION  
This project implements three modes of control used to navigate indoors. These modes can be used to carry out a tour of a building using on floor line navigation, colored object following, and manual control. More about this project can be found on our website at https://tourdrone.github.io. Demos of the project working, as well as some of our attempts along the way, can be found at https://www.youtube.com/channel/UCb4gyYxqb0HRZoYWGTBV6GA. 
    
# INSTALLATION  
The code runs on Ubuntu 16.4.01 LTS, which can be loaded and run using VMware Fusion. Additionally, this project runs on a Raspberry Pi 3.0, again using Ubuntu 16.4.01 LTS. It is intended as a Linux install.

### Linux install instructions  
  1. Clone the repository  
  2. Sudo apt-get install these dependencies  
  	ffmpeg:  
	  FFmpeg 2.2.3 (LGPL v2.1 license) http://www.ffmpeg.org/  
	libopencv-dev:  
	  OpenCV 3.0.0 Alpha (BSD license) http://opencv.org/  
  3. Run the makefile in build/linux  
  
### Mac install instructions  
  1. Clone the repository  
  2. Sudo apt-get install both the  
  	ffmpeg:  
	  FFmpeg 2.2.3 (LGPL v2.1 license) http://www.ffmpeg.org/  
	libopencv-dev:  
	  OpenCV 3.0.0 Alpha (BSD license) http://opencv.org/  
  3. Run the makefile in build/linux  
  4. Additional modification of the dependency libraries may be required.  

# EXECUTION  
Run ./test.a form the build/linux folder  

# CONTROL  
  Keyboard input is used to control this project.  
  
### Default
  Takeoff/Land: Spacebar  
  Set Speed: 0 to 9  
  Increase Speed: +  
  Decrease Speed: -  
  Switch Modes: v  
  Select Manual Mode: b  
  Select Object Following Mode: n  
  Select Line Following Mode: m  
  
### Manual  
  Up: q  
  Down: a  
  Forwards: t  
  Backwards: g  
  Left: f  
  Right: h  
  Rotate Left: r  
  Rotate Right: y  
  
### Object Following  
  Learn Object Color: l  
  
### Line Following  
  Learn Line Color: l  

# LEGAL  
This project was originally forked from https://github.com/tekkies/cvdrone. We have included the license in that readme. 
	  
This is free software; you can redistribute it and/or modify it under the terms of EITHER:  
  (1) The GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version. The text of the GNU Lesser General Public License is included with this library in the file cvdrone-license-LGPL.txt.  
  (2) The BSD-style license that is included with this library in the file cvdrone-license-BSD.txt.  

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.  
