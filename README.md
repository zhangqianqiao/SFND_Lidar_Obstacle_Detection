# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

### Project Starter Repository
The source code for the project is contained in ***src*** directory;
The src directory contains the following files:

 **render**
 **sensors**
 *environemnt.cpp*
 *processPointClouds.h*
 *processPointClouds.cpp*

__RANSAC__
The RANSAC algorithms is in the function ProcessPointClouds::SegmentPlane of the file ***processPointClouds.cpp***;

__KD-Tree__
The interface of the KD_Tree is implemented in the file ***processPointClouds.h***;

__Euclidean clustering__
The Euclidean clustering algorithms is in the function ProcessPointClouds::euclideanCluster and ProcessPointClouds::clusterHelper of the file ***processPointClouds.cpp***;

### Compiling and Running

#### Compiling
To compile the project, first, create a build directory and change to that directory:
	mkdir build && cd build 
From within the build directory, then run cmake and make as follows:
	cmake ..
	make
#### Running
The executable will be placed in the build directory. From within build, you can run the project as follows:
	./environment

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
