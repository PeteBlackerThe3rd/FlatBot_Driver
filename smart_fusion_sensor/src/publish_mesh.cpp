#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <gncTK.h>
#include "gnc_tool_kit/FusedSurface.h"

int main(int argc, char** argv)
{
  if (argc < 2)
  {
	  printf("Not enough parameters given.\nUsage: <node> <obj file name> optional<frequency Hz>\n");
	  exit(1);
  }

  float Hz = 0.1;
  if (argc >= 3)
  {
	  float newHz = atof(argv[2]);
	  if (newHz > 0)
	  {
		  Hz = newHz;
		  printf("publish frequency set to %f Hz\n", Hz);
	  }
  }

  ros::init(argc, argv, "mesh_publisher_from_file_node");
  ros::NodeHandle nh;

  /*cv_bridge::CvImage cv_image;
  cv_image.image = cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
  cv_image.encoding = "bgr8";
  sensor_msgs::Image ros_image;
  cv_image.toImageMsg(ros_image);*/

  gncTK::Mesh mesh = gncTK::Mesh::loadOBJ(std::string(argv[1]));
  mesh.frameId = "world";

  ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("/static_marker", 1);
  ros::Publisher meshPub = nh.advertise<gnc_tool_kit::FusedSurface>("/static_mesh", 1);
  ros::Rate loop_rate(Hz);

  while (nh.ok())
  {
    //pub.publish(ros_image);

	markerPub.publish(mesh.toMarkerMsg(gncTK::Mesh::ColorTexture));
	meshPub.publish(mesh.toFusedSurfaceMsg());

	ROS_INFO("Published marker and mesh messages.");

    loop_rate.sleep();
  }
}
