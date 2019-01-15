/*-----------------------------------------------------------\\
||                                                           ||
||                 LIDAR fusion GNC project                  ||
||               ----------------------------                ||
||                                                           ||
||    Surrey Space Centre - STAR lab                         ||
||    Planetary Robotics PhD                                 ||
||    (c) Surrey University 2017                             ||
||    Pete dot Blacker at Gmail dot com                      ||
||                                                           ||
\\-----------------------------------------------------------//

smart_rover_sensor.h

Object to control the fusion hybrid sensor on the smart rovers
-----------------------------------------

This object handles controlling the pan tilt unit, and collects
data from the lidar and camera. This data is broadcast as
a structured point cloud and an image along with a coordinate
frame describing the position of the sensor when the scan was
collected.

Topics avilable are:
--------------------

sme_awareness/bare-mesh
sme_awareness/bare-simplified-mesh
sme_awareness/color-mesh
sme_awareness/color-simplified-mesh
sme_awareness/scan-progress-monitor

Services available are:
-----------------------
sme_awareness/scan

-------------------------------------------------------------*/
#ifndef SMART_FUSION_SENSOR_SENSOR_H_
#define SMART_FUSION_SENSOR_SENSOR_H_

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/ColorRGBA.h"
#include "image_transport/image_transport.h"
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>

// ROS messages and services
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_listener.h"
#include "dp_ptu47_msgs/SendCommand.h"
#include "dp_ptu47_msgs/SendTrajectory.h"
#include "dp_ptu47_msgs/GetLimits.h"
#include "dp_ptu47_msgs/PanTiltStamped.h"
#include "smart_fusion_sensor/PerformScan.h"

class SmartRoverSensor
{
public:

	SmartRoverSensor(ros::NodeHandle n,
					 std::string cameraTopic = "/GoPro/image_raw",
					 std::string lidarTopic = "scan",
					 std::string _sensorFrameId = "fusion_sensor");

	void startScanService();

	bool performScan(float maxAngle, float minAngle, float panAngle);

	// start up the scan service handler
	void startScanService(ros::NodeHandle n);

	// method to handle an external scan request
	bool scanServiceCallback(smart_fusion_sensor::PerformScan::Request &req,smart_fusion_sensor::PerformScan::Response &resp);

	void broadcastBaseToSensorTransform();

	void broadcastEmptyProgressCloud();

	void (*fusionCompleteCallback)(cv::Mat image, pcl::PointCloud<pcl::PointXYZI> points);

private:

  void ptuListener(const dp_ptu47_msgs::PanTiltStamped::ConstPtr &Position);

  // Image capture callback
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // function to finish off a scan
  void finishAndWriteScan();

  // function to add a line of lidar data into the cumulative scan array
  void addLidarLine(const sensor_msgs::LaserScan::ConstPtr& scan);

  // Lidar scan callback listener
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  // method to move the pan tilt unit to a given tilt angle
  bool tiltTo(float angle, float panAngle);

  // method send a slow trajectory command to the pan tilt unit
  bool tiltTrajectoryTo(float angle, float panAngle);

  // global listener and publisher objects
  tf::TransformListener *tfListener;
  ros::Publisher progressCloudPublisher;

  // service client objects for scan control and scan trajectory
  ros::ServiceClient client_sc;
  ros::ServiceClient client_st;

  // service server object for scan service
  ros::ServiceServer scanService;

  std::string sensorFrameId;
  std::string baseFrameId;
  std::string lidarFrameId;

  // scan data structure
  pcl::PointCloud<pcl::PointXYZI> lidarPoints;
  geometry_msgs::PointStamped scanOrigin;
  bool imagesReceived;
  cv::Mat scanImage;
  ros::Time scanMidStamp;
  tf::StampedTransform baseToSensorTransform;
  bool baseToSensorSet;

  // scanning status
  enum scanStat { SCANNING, NOT_SCANNING };
  scanStat scanStatus;
  //double lastHeight;
  bool waitingForPicture;

  ros::Subscriber scanSub;
  ros::Subscriber ptuSub;

  image_transport::Subscriber itSub;

  float tiltAngle;
  float deltaTiltAngle;
};

#endif /* SMART_FUSION_SENSOR_SENSOR_H_ */
