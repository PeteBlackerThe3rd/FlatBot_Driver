/*-----------------------------------------------------------\\
||                                                           ||
||                 LIDAR fusion GNC project                  ||
||               ----------------------------                ||
||                                                           ||
||    Surrey Space Centre - STAR lab                         ||
||    Space Engineering MSc disseration                      ||
||    (c) Pete Blacker 2016                                  ||
||    Pete dot Blacker at Gmail dot com                      ||
||                                                           ||
\\-----------------------------------------------------------//

senor_test.cpp

Entry point of the fusion sensor testing ros node.
-----------------------------------------

-------------------------------------------------------------*/
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "smart_rover_sensor.h"
#include "gncTK.h"
#include "gnc_tool_kit/FusedSurface.h"

// structured lidar fusion processing object
gncTK::FusionStructured fusion;

// Path to save scans to. Empty string indicates no files should be saved.
std::string scanSavePath = "";

ros::Publisher meshPublisher;
ros::Publisher fusedSurfacePublisher;
image_transport::Publisher depthPublisher;

void broadcastImage(cv::Mat image, image_transport::Publisher publisher)
{
	// publish image with detected channels
	cv_bridge::CvImage cvBridgeImg;
	if (image.channels() == 3)
		cvBridgeImg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	else if (image.channels() == 1)
		cvBridgeImg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

	cvBridgeImg.image = image;
	publisher.publish(cvBridgeImg.toImageMsg());
}

void scanComplete(cv::Mat image, pcl::PointCloud<pcl::PointXYZI> points)
{
	static int scanNumber = 0;

	printf("Scan sensor collection complete."); fflush(stdout);

	// create fused mesh object and broadcast
	fusion.setInputCloud(points);
	fusion.setInputImage(image);
	gncTK::Mesh fusedMesh = fusion.generateMesh();

	printf("Fused mesh generated."); fflush(stdout);

	meshPublisher.publish(fusedMesh.toMarkerMsg());

	printf("Marker Message published"); fflush(stdout);

	fusedSurfacePublisher.publish(fusedMesh.toFusedSurfaceMsg());

	printf("Fused Surface message published"); fflush(stdout);

	/*cv::Mat depth = fusion.generateDepthImage();
	depth = gncTK::Utils::floatImageTo8Bit(depth, 0);
	broadcastImage(depth, depthPublisher);

	printf("Depth Image Published"); fflush(stdout);*/

	if (scanSavePath != "")
	{
		printf("Saving fusion image and point cloud number %05d to path [%s]\n", scanNumber, scanSavePath.c_str());

		char fileName[256];
		sprintf(fileName, "%s/camera_%05d.png", scanSavePath.c_str(), scanNumber);
		imwrite(fileName, image);

		sprintf(fileName, "%s/points_%05d.pcd", scanSavePath.c_str(), scanNumber);
		pcl::io::savePCDFileASCII(fileName, points);

		sprintf(fileName, "%s/mesh_%05d", scanSavePath.c_str(), scanNumber);
		fusedMesh.saveOBJ(std::string(fileName));
	}

	++scanNumber;
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ Starting lidar fusion node ]--");

	// setup ros for this node and get handle to ros system
	ros::init(argc, argv, "lidar_scan");
	ros::start();

	// get handle for this node
	ros::NodeHandle n;
    image_transport::ImageTransport imgTrans(n);

	// create a sensor object for this node
	SmartRoverSensor *fusionSensor = new SmartRoverSensor(n);

	// setup fused mesh publisher
	meshPublisher = n.advertise<visualization_msgs::Marker>("lidar_surface",10);

	// setup fused surface (textured plugin) publisher
	fusedSurfacePublisher = n.advertise<gnc_tool_kit::FusedSurface>("textured_surface",10);

	// setup depth image publisher
    depthPublisher = imgTrans.advertise("lidar_depth", 1);

	// load fusion sensor calibration settings
	ros::NodeHandle nodeP("~");
	std::string calibrationFileName;
	nodeP.param("config_file", calibrationFileName, std::string("default_calibration.csv"));

	nodeP.param("scan_save_path", scanSavePath, std::string(""));

	fusion.setFusionFunction( gncTK::FusionFunction(calibrationFileName) );

    // initial demonstration scan
	ROS_INFO("Performing initial test scan.");
    fusionSensor->performScan(25, -40, 0);

    // setup scan service listener and callback to fusion function
    ROS_INFO("Setting up lidar scan service listener");
    fusionSensor->startScanService(n);
    fusionSensor->fusionCompleteCallback = scanComplete;

    ROS_INFO("--[ lidar-scanner node: startup complete ]--");

    ros::Rate loopRate(100);
    while(ros::ok())
    {
    	fusionSensor->broadcastBaseToSensorTransform();
    	ros::spinOnce();
    	loopRate.sleep();
    }

    ROS_INFO("LIDAR scanning node shutdown OK.\n");
	return 0;
}
