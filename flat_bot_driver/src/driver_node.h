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
#include <stdlib.h>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "flat_bot_msgs/SetStatus.h"

class FlatBotDriver
{
public:

	ros::NodeHandle n;
	actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction> trajectoryActionServer;

	FlatBotDriver();

	// start up the scan service handler
	//void startScanService(ros::NodeHandle n);

	// method to handle an external scan request
	//bool scanServiceCallback(smart_fusion_sensor::PerformScan::Request &req,smart_fusion_sensor::PerformScan::Response &resp);

	//void (*fusionCompleteCallback)(cv::Mat image, pcl::PointCloud<pcl::PointXYZI> points);

	void publishStatus(const ros::TimerEvent&);

	bool setStatus(flat_bot_msgs::SetStatus::Request  &req,
    		  	   flat_bot_msgs::SetStatus::Response &res);

	void publishJointState(const ros::TimerEvent&);

	void coreLoop(const ros::TimerEvent&);

	void trajectoryHandler(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal);

	class Joint
	{
	public:
		Joint(std::string name, float initPos, float minPos, float maxPos)
		{
		  this->name = name;
		  this->pos = initPos;
		  this->minPos = minPos;
		  this->maxPos = maxPos;
		}

		std::string name;
		float pos, minPos, maxPos;
	};

private:

  // driver status publisher and timer
  ros::Publisher statusPublisher;
  ros::Timer statusTicker;

  // joint state publisher and timer
  ros::Publisher jointStatePublisher;
  ros::Timer jointTicker;

  // core control loop timer
  ros::Timer coreTimer;

  // service server object for status change requests
  ros::ServiceServer setStatusService;

  // action server to handle Trajectory requests


  std::string hardwarePortName;

  std::vector<Joint> joints;

  // status of the arm
  enum Status { DISABLED, LIVE_SIM, LIVE_HARDWARE };
  Status status;

  std::vector<trajectory_msgs::JointTrajectoryPoint> activeTrajectory;
  ros::Time trajectoryStartTime;
  enum ArmStatus { ARM_STATIC, ARM_FOLLOWING_TRAJECTORY };
  ArmStatus armStatus;

  enum HardwareStatus { HARDWARE_CONNECTED, HARDWARE_DISCONNECTED };
  HardwareStatus hardwareStatus;
};

#endif /* SMART_FUSION_SENSOR_SENSOR_H_ */
