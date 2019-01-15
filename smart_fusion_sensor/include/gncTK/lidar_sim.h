#ifndef GNCTK_LIDAR_SIM_H_
#define GNCTK_LIDAR_SIM_H_

/*-----------------------------------------------------------\\
||                                                           ||
||                 LIDAR fusion GNC project                  ||
||               ----------------------------                ||
||                                                           ||
||    Surrey Space Centre - STAR lab                         ||
||    (c) Surrey University 2017                             ||
||    Pete dot Blacker at Gmail dot com                      ||
||                                                           ||
\\-----------------------------------------------------------//

lidar_sim.cpp

LIDAR sensor simulator node
----------------------------

This node uses a pair of depth and incident cube map images
to simulate the data from a LIDAR sensor within the
cube maps.

-------------------------------------------------------------*/
#include <stdio.h>
#include "cv.h"
#include <pcl_ros/point_cloud.h>
#include "opencv2/highgui/highgui.hpp"

namespace gncTK {
class LidarSim;
};

class gncTK::LidarSim
{
public:

	LidarSim();

	struct lidarParams
	{
	  float beamDivergence;
	  // energy to depth error fn
	  // energy to sample probablility fn
	};

	struct cubeSample
	{
	  float depth;
	  float incident;
	  float albedo;
	  float mask;
	};

	Eigen::Vector2f getSampleLocation(Eigen::Vector3f direction);

	cubeSample generateCubeSample(Eigen::Vector3f direction);

	pcl::PointXYZI generateLidarSample(Eigen::Vector3f direction);

	pcl::PointCloud<pcl::PointXYZI> generateScan(Eigen::Matrix3f orientation);

	void loadCubeMaps(char *prefix);

protected:
	bool mapsLoaded;
	cv::Mat depthMap, incidentMap;
	int mapResolution;

	int multisampleCount;
	float maxDepth;

	cv::Mat mapCopy;
	cv::Vec3b debugColor;
};





#endif /* GNCTK_LIDAR_SIM_H_ */
