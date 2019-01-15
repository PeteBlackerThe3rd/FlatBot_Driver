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

smart_rover_sensor.cpp

Object to control the fusion hybrid sensor on the smart rovers
-----------------------------------------

This object handles controlling the pan tilt unit, and collects
data from the lidar and camera. This data is broadcast as
a structured point cloud and an image along with a coordinate
frame describing the position of the sensor when the scan was
collected.

Topics avilable are:
--------------------

Services available are:
-----------------------
smart_fusion_sensor/scan

-------------------------------------------------------------*/
#include "smart_rover_sensor.h"

#include <time.h>  // used for measuring CPU time of various algorithms
#include <sys/times.h>

// default constructor
//SmartRoverSensor::SmartRoverSensor()
//{
//}

// constructor
SmartRoverSensor::SmartRoverSensor(ros::NodeHandle n,
								   std::string cameraTopic,
								   std::string lidarTopic,
								   std::string _sensorFrameId)
{

  // get sensor parameters from ros if given
  ros::NodeHandle nodeP("~");
  std::string progressPointCloudTopicName;
  std::string panTiltBaseTopic;

  nodeP.param("camera_topic", cameraTopic, cameraTopic);
  nodeP.param("lidar_topic", lidarTopic, lidarTopic);
  nodeP.param("sensor_frame_id", sensorFrameId, _sensorFrameId);
  nodeP.param("body_frame_id", baseFrameId, std::string("body_frame"));
  nodeP.param("progress_cloud_topic", progressPointCloudTopicName, std::string("progress_point_cloud"));
  nodeP.param("pan_tilt_base_topic", panTiltBaseTopic, std::string("/dp_ptu47"));
  nodeP.param("lidar_frame_id", lidarFrameId, std::string("/laser_tilt_link"));

  imagesReceived = false;
  waitingForPicture = false;
  baseToSensorSet = false;
  float tiltAngle=0;
  float deltaTiltAngle=0;
  scanStatus = NOT_SCANNING;
  fusionCompleteCallback = NULL;

  client_sc = n.serviceClient<dp_ptu47_msgs::SendCommand>(panTiltBaseTopic + "/control");
  client_st = n.serviceClient<dp_ptu47_msgs::SendTrajectory>(panTiltBaseTopic + "/control_trajectory");
  ros::ServiceClient client_gl = n.serviceClient<dp_ptu47_msgs::GetLimits>(panTiltBaseTopic + "/get_limits");

  progressCloudPublisher = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(progressPointCloudTopicName, 10);

  ros::Duration tfCacheDuration;
  tfCacheDuration = tfCacheDuration.fromSec(60);
  tfListener = new tf::TransformListener(tfCacheDuration);

  scanSub = n.subscribe(lidarTopic.c_str(), 300, &SmartRoverSensor::scanCallback,this);
  ptuSub = n.subscribe(panTiltBaseTopic + "/pan_tilt_status_stamped", 100, &SmartRoverSensor::ptuListener,this);
  image_transport::ImageTransport it(n);
  itSub = it.subscribe(cameraTopic.c_str(),1,&SmartRoverSensor::imageCallback,this,image_transport::TransportHints("compressed"));
}


// start up the scan service handler
void SmartRoverSensor::startScanService(ros::NodeHandle n)
{
  scanService = n.advertiseService("smart_fusion_sensor/PerformScan", &SmartRoverSensor::scanServiceCallback, this);
}

void SmartRoverSensor::ptuListener(const dp_ptu47_msgs::PanTiltStamped::ConstPtr &Position)
{
  deltaTiltAngle = tiltAngle - Position->tilt_angle;
  tiltAngle = Position->tilt_angle;
}

// Image capture callback
void SmartRoverSensor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // if we're currently waiting for the picture to be taken
  if (waitingForPicture)
    {
    //ROS_INFO("Scan Image Captured.");

    // copy this image and add to the mesh
    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, "bgr8");
    scanImage = cvImg->image;

    imagesReceived = true;
    waitingForPicture = false;
    }
}

// function to finish off a scan
void SmartRoverSensor::finishAndWriteScan() {

  if (!imagesReceived)
  {
    ROS_INFO("Ignoring scan with no camera image");
    return;
  }

  if (lidarPoints.height >= 10)
  {
	  if (fusionCompleteCallback != NULL)
		  fusionCompleteCallback(scanImage, lidarPoints);
  }
  else
    ROS_INFO("Ignoring scan of [%d] lines.", lidarPoints.height);
}

// function to add a line of lidar data into the cumulative scan array
void SmartRoverSensor::addLidarLine(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	geometry_msgs::PointStamped scanFramePoint;
	geometry_msgs::PointStamped baseFramePoint;

	// set minimum distance to an object will be recorded
	float lidarMinDistance = 0.15; // 15 cm

	// set lidar FOV in degrees
	float hFOV = 90;//   <----------------------------
	hFOV /= 180/3.141592654;

	double minAngle = scan->angle_min;
	std::vector<float> ranges(scan->ranges);
	std::vector<float> intensities(scan->intensities);

	// filter out samples that are outside of the chosen FOV
	for (int i=0; i<ranges.size(); ++i)
	{
		double currentAngle = minAngle + (i * scan->angle_increment);

		if (currentAngle < (0 - hFOV/2))
		{
			minAngle = currentAngle + scan->angle_increment;
			ranges.erase(ranges.begin());
			intensities.erase(intensities.begin());
			--i;
		}
		if (currentAngle > hFOV/2)
		{
			ranges.erase(ranges.begin() + i, ranges.end());
			intensities.erase(intensities.begin() + i, intensities.end());
			break;
		}
	}

	// enlarge the point cloud to include this new row
	lidarPoints.width = scan->ranges.size();
	++lidarPoints.height;
	lidarPoints.points.resize(lidarPoints.width * lidarPoints.height);

	for (int i=0; i<ranges.size(); ++i)
	{
		double currentAngle = minAngle + (i * scan->angle_increment);
		pcl::PointXYZI newPoint;

		double setMax = 80.0; // currently manually set maximum range of the hukuyo sensor
							  // the range_max value in the scan message isn't corrent

		// Check range is within measurable range this picks up null samples
		if (ranges[i] >= scan->range_min &&
		    ranges[i] >= lidarMinDistance &&
		    ranges[i] <= scan->range_max &&
			ranges[i] <= setMax)
		{
			// setup time and reference frame for this point
			scanFramePoint.header.frame_id = scan->header.frame_id;
			scanFramePoint.header.stamp = scan->header.stamp;

			// calculate point in cartesian coordinates
			scanFramePoint.point.y = sin(currentAngle) * ranges[i];
			scanFramePoint.point.x = cos(currentAngle) * ranges[i];
			scanFramePoint.point.z = 0;

			// transform into sensor reference frame and save to point cloud
			tfListener->transformPoint(sensorFrameId.c_str(), scanFramePoint, baseFramePoint);
			newPoint.x = baseFramePoint.point.x;
			newPoint.y = baseFramePoint.point.y;
			newPoint.z = baseFramePoint.point.z;

			// if the lidar is setup to return intensity values then
			// add the intensity of this sample to the vertex
			if (intensities.size() != 0)
				newPoint.intensity = intensities[i];
			else
				newPoint.intensity = 0;
		}
		else // add null vertex if this scan value was out of range
		{
			newPoint.x = newPoint.y = newPoint.z = NAN;
		}

		int pointIndex = (lidarPoints.width * (lidarPoints.height-1)) + i;
		lidarPoints.points[pointIndex] = newPoint;
	}

	progressCloudPublisher.publish(lidarPoints.makeShared());
}

void SmartRoverSensor::broadcastEmptyProgressCloud()
{
	pcl::PointCloud<pcl::PointXYZI> emptyPoints;
	emptyPoints.width = 0;
	emptyPoints.height = 0;
	emptyPoints.is_dense = false;
	emptyPoints.header.frame_id = sensorFrameId.c_str();
	progressCloudPublisher.publish(emptyPoints.makeShared());
}

// Lidar scan callback listener
void SmartRoverSensor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if (scanStatus != SCANNING)
		return;

	// wait for the required transform to become available
	tfListener->waitForTransform(baseFrameId.c_str(), scan->header.frame_id, scan->header.stamp, ros::Duration(0.5));

	// if we're scanning and there's still space in the scan
	if (deltaTiltAngle > 0)
	{
		try
		{
			printf("\rProcessing line [%d] of scan.", lidarPoints.height+1);
			fflush(stdout);

			addLidarLine(scan);
		}
		catch (tf2::TransformException &ex)
		{
			ROS_INFO("Exception thrown getting lidar transformation [%s]", ex.what());
		}
	}
}

// method to handle an external scan request
bool SmartRoverSensor::scanServiceCallback(smart_fusion_sensor::PerformScan::Request &req, smart_fusion_sensor::PerformScan::Response &resp)
{
  ROS_INFO("Scan Service Call requested");

  // validate the given scan angles (max 25 min 40) must be at least 10 degrees

  // scan must be at least 10 degrees high
  if (req.maxAngle < req.minAngle + 10)
  {
	ROS_INFO("couldn't perform scan smalled than 10 degrees [%f,%f]", req.minAngle, req.maxAngle);
    resp.startedOk = false;
    return true;
  }

  if (req.minAngle < -40 || req.maxAngle > 25)
  {
	ROS_INFO("couldn't perform scan out of range max is 25 min is -40 [%f,%f]", req.minAngle, req.maxAngle);
    resp.startedOk = false;
    return true;
  }

  if (req.minPanAngle == req.maxPanAngle)
  {
    if (req.minPanAngle < -155 || req.minPanAngle > 155)
    {
	  ROS_INFO("couldn't perform scan, out of range pan angle given. range is [-155 to +155] degrees");
	  resp.startedOk = false;
	  return true;
    }

    performScan(req.maxAngle, req.minAngle, req.minPanAngle);

    ROS_INFO("Completed single scan.");
  }
  else
  {
	if (req.minPanAngle > req.maxPanAngle)
	{
	  float t = req.minPanAngle;
	  req.minPanAngle = req.maxPanAngle;
	  req.maxPanAngle = t;
	}

	int maxAngleBetweenScans = 90; // in degrees

	int pans = ceil((req.maxPanAngle - req.minPanAngle) / maxAngleBetweenScans);
	float panDAngle = (req.maxPanAngle - req.minPanAngle)/pans;

	for (int s=0; s<= pans; ++s)
	{
	  performScan(req.maxAngle, req.minAngle, req.minPanAngle + (s * panDAngle));
	}

	ROS_INFO("Completed %d scans.", pans+1);
  }

  resp.startedOk = true;

  return true;
}

void SmartRoverSensor::broadcastBaseToSensorTransform()
{
	static tf::TransformBroadcaster br;

	if (baseToSensorSet)
	{
		//printf("broadcast base to sensor.\n");
		baseToSensorTransform.stamp_ = ros::Time::now();
		br.sendTransform(baseToSensorTransform);
	}
}

// method to move the pan tilt unit to a given tilt angle
bool SmartRoverSensor::tiltTo(float angle, float panAngle)
{
  dp_ptu47_msgs::SendCommand srv_sc;
  srv_sc.request.pan_angle = panAngle;
  srv_sc.request.tilt_angle = angle;
  srv_sc.request.wait_finished = true;
  if (client_sc.call (srv_sc))
	{
	ROS_INFO ("Dp_ptu service call successful. New pan/tilt positions are: %f/%f.", srv_sc.response.pan_angle, srv_sc.response.tilt_angle);
	}
  else
	{
	ROS_ERROR ("Failed to call dp_ptu command service [%s]", client_sc.getService().c_str());
	if (client_sc.exists())
		ROS_ERROR("Service exists and is avaialble.");
	else
		ROS_ERROR("Service either doesn't exist or is not avilable.");
	return false;
	}
  return true;
}

// method send a slow trajectory command to the pan tilt unit
bool SmartRoverSensor::tiltTrajectoryTo(float angle, float panAngle)
{
  std::vector<float> pan_angles;
  std::vector<float> tilt_angles;
  pan_angles.push_back(panAngle);
  tilt_angles.push_back(angle);

  dp_ptu47_msgs::SendTrajectory srv_st;
  srv_st.request.pan_angles = pan_angles;
  srv_st.request.tilt_angles = tilt_angles;
  srv_st.request.tolerance = 2;
  if (client_st.call (srv_st))
	{
	ROS_INFO ("Trajectory service call successful. New pan/tilt positions are: %f/%f.", srv_st.response.pan_angle, srv_st.response.tilt_angle);
	}
  else
	{
	ROS_ERROR ("Failed to call trajectory service [%s]", client_st.getService().c_str());
	if (client_st.exists())
		ROS_ERROR("Service exists and is available.");
	else
		ROS_ERROR("Service either doesn't exist or is not available.");
	return false;
	}
  return true;
}

bool SmartRoverSensor::performScan(float maxAngle, float minAngle, float panAngle)
{
  //---------------------------------------------------
  ROS_INFO("move to mid scan position to take picture");
  if (!tiltTo((maxAngle+minAngle)/2, panAngle))
	  return false;

  // pause for a for TF system to catch up
  ros::Duration(1).sleep();

  // save transformation from body frame to mid scan position
  // this will be the reference frame for the scan data
  printf("looking up transform from [%s] -> [%s]\n", baseFrameId.c_str(), lidarFrameId.c_str());
  tfListener->lookupTransform(baseFrameId.c_str(), lidarFrameId.c_str(), ros::Time(0), baseToSensorTransform);
  baseToSensorTransform.child_frame_id_ = sensorFrameId;
  baseToSensorSet = true;

  //std::cout << "base to sensor recorded. is:\n" << baseToSensorTransform << "\n------\n";
  printf("base to sensor recorded.\n");

  // empty and setup point cloud object ready for new scan
  lidarPoints.points.clear();
  lidarPoints.width = 0;
  lidarPoints.height = 0;
  lidarPoints.is_dense = false;
  lidarPoints.header.frame_id = sensorFrameId.c_str();

  // take picture for scan texturing
  waitingForPicture = true;
  imagesReceived = false;
  while(waitingForPicture)
	  ros::spinOnce();

  ROS_INFO("move to scan start position");
  if (!tiltTo(maxAngle, panAngle))
	  return false;

  // wait for the sensor mast to stop wobbling!
  ros::Duration(3).sleep();

  //---------------------------------------------------
  ROS_INFO("Sending trajectory to scan the ptu");
  if (!tiltTrajectoryTo(minAngle, panAngle))
	return false;

  // listen to scan and transform message as platform tilts.
  ros::Rate msgRate(100);
  scanStatus = SCANNING;
  ros::Duration timeOut(15);
  ros::Time start = ros::Time::now();
  while (tiltAngle >(minAngle+1) && ros::Time::now() < start+timeOut)
  {
	  broadcastBaseToSensorTransform();

	  ros::spinOnce();

	  msgRate.sleep();
  }
  scanStatus = NOT_SCANNING;

  printf("\nScan Complete.\n");

  printf("returning to mid scan position");
  fflush(stdout);
  if (!tiltTrajectoryTo((maxAngle+minAngle)/2, panAngle))
	  return false;
  ros::Duration(1).sleep();

  // now finish the scan
  finishAndWriteScan();
  broadcastEmptyProgressCloud();

  printf("Finished Scan");
  fflush(stdout);

  return true;
}
