#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
	  printf("Not enough parameters given.\nUsage: <node> <image file name> optional<frequency Hz>\n");
	  exit(1);
  }

  float Hz = 1;
  if (argc >= 3)
  {
	  float newHz = atof(argv[2]);
	  if (newHz > 0)
	  {
		  Hz = newHz;
		  printf("publish frequency set to %f Hz\n", Hz);
	  }
  }

  ros::init(argc, argv, "image_publisher_from_file_node");
  ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);
  ros::Rate loop_rate(Hz);

  while (nh.ok())
  {
    pub.publish(ros_image);
    loop_rate.sleep();
  }
}
