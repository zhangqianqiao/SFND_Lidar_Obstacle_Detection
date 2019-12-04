# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

### Project Starter Repository
The source code for the project is contained in ***src*** directory;
The src directory contains the following files:

 **render**  
 **sensors**  
 *environemnt.cpp*  
 *processPointClouds.h*  
 *processPointClouds.cpp*  

__RANSAC__  
The RANSAC algorithms is in the function ProcessPointClouds::SegmentPlane of the file ***processPointClouds.cpp*** ;  

__KD-Tree__  
The interface of the KD_Tree is implemented in the file ***processPointClouds.h*** ;

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
