
#include <math.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
//#include <actionlib/server/simple_action_server.h>
//#include "moveit_msgs/ExecuteTrajectoryAction.h"

#include "herkulex_interface.h"

Herkulex::Interface *hInterface;

sensor_msgs::JointState getJointStateMsg(Herkulex::Interface *interface, std::vector<std::string> jointNames)
{
	static int seqId = 0;

	sensor_msgs::JointState jointStateMsg;

	jointStateMsg.header.seq = seqId++;
	jointStateMsg.header.stamp = ros::Time::now();
	jointStateMsg.header.frame_id = "";

	jointStateMsg.name = jointNames;

	std::vector<Herkulex::ServoJointStatus> joints = interface->getJointStates();

	for (int j=0; j<joints.size(); ++j)
	{
		jointStateMsg.position.push_back(joints[j].angleRads);
		jointStateMsg.velocity.push_back(joints[j].velocityRadsPerSec);
	}

	return jointStateMsg;
}

void positionCallback(std_msgs::Float32 msg)
{
	float value = msg.data;
	printf("value from msg was [%f]\n", value);

	Herkulex::TrajectoryPoint test(219, 1.0, 3.141);
	test.setDegrees(value);
	hInterface->jogServo(test);
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ Herkulex Test Node ]--");

	// setup ros for this node and get handle to ros system
	ros::init(argc, argv, "Herkulex_test");
	ros::start();

	ros::NodeHandle n("~");
	std::string portName, jointTopic, jointNamesParam;
	int BAUDRate, jointPublisherRate;
	n.param<std::string>("herkulex_port", portName, "/dev/ttyUSB0");
	n.param<int>("herkulex_baud", BAUDRate, 115200);
	n.param<int>("joint_publish_rate", jointPublisherRate, 20);
	n.param<std::string>("joint_state_topic", jointTopic, "joint_states");
	n.param<std::string>("joint_names", jointNamesParam, "");

	ros::Publisher jointPublisher = n.advertise<sensor_msgs::JointState>(jointTopic, 10);

	ros::Subscriber sub = n.subscribe("/position", 10, positionCallback);

	ROS_INFO("connecting to Herkulex servo string on port \"%s\" with BAUD %d", portName.c_str(), BAUDRate);
	Herkulex::Interface interface(portName, BAUDRate);
	hInterface = &interface;

	if (!interface.isConnected())
	{
		ROS_ERROR("Error : Failed to setup Herkulex interface [%s]", interface.getConnectionError().c_str());
		return 1;
	}

	// write detected servo info to log
	std::vector<Herkulex::Servo> servos = interface.getServos();
	ROS_INFO("Detected %d herculex servos :", (int)servos.size());
	for (int s=0; s<servos.size(); ++s)
		ROS_INFO("%s", servos[s].toString().c_str());

	// setup joint names
	std::vector<std::string> jointNames;
	if (jointNamesParam == "")
	{
		for (int s=0; s<servos.size(); ++s)
			jointNames.push_back("Joint_" + std::to_string(s+1));
	}
	else
	{
		boost::split(jointNames, jointNamesParam, [](char c){return c == ' ';});

		if (jointNames.size() != servos.size())
		{
			ROS_ERROR("Error: Incorrect number of joint names specified. %d servos and %d names.",
					  (int)servos.size(),
					  (int)jointNames.size());
		}
	}

	ROS_INFO("Joint names set to:");
	for (int j=0; j<jointNames.size(); ++j)
		ROS_INFO("  [%3d] - %s", j, jointNames[j].c_str());

    ROS_INFO("--[ Herkulex Test Node: startup complete ]--");

    int servoId = servos[0].id;
    interface.setLed(servoId, LED_CONTROL_BLUE);
    interface.setTorqueMode(servoId, TORQUE_CONTROL_TORQUE_ON);

    Herkulex::TrajectoryPoint test(servoId, 2.0, 3.141);
    test.setDegrees(-95.0);
    interface.jogServo(test);

    ros::Rate loopRate(jointPublisherRate);
    int seq=0;
    while(ros::ok())
    {
    	sensor_msgs::JointState jointStateMsg = getJointStateMsg(&interface, jointNames);
    	jointPublisher.publish(jointStateMsg);

    	/*seq = (seq++) % 20;
    	if (seq == 0)
    	{
    		Herkulex::ServoPowerStatus power = interface.getServoPowerStatus(servoId);
    		ROS_INFO("Servo [%d] temp %d degrees, voltage %f v",
    				 servoId,
					 power.tempDegrees,
					 power.voltageVolts);
    	}*/

    	ros::spinOnce();
    	loopRate.sleep();
    }

	printf("--[ Herkulex Test Node: shutdown OK ]--\n");
	return 0;
}
