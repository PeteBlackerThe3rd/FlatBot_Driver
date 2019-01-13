#include <math.h>
#include <stdlib.h>

//#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
//#include <boost/asio/buffer.hpp>

// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "flat_bot_msgs/SetStatus.h"

boost::asio::serial_port *port;

#define EEP_WRITE (u_char) 0x01 // command for writing to EEP
#define EEP_READ (u_char) 0x02// command for reading from EEP
#define RAM_WRITE (u_char) 0x03 // command for writing to RAM
#define RAM_READ (u_char) 0x04 // command for reading from RAM
#define I_JOG (u_char) 0x05 // command for servo movement control
#define S_JOG (u_char) 0x06 // command for servo movement control
#define STAT (u_char) 0x07 // command for getting servo status
#define ROLLBACK (u_char) 0x08 // command for restoring factory defaults
#define REBOOT (u_char) 0x09 // command for rebooting servo

#define ACK_POLICY_RAM_ADDR (u_char) 0x01
#define ACK_POLICY_EEP_ADDR (u_char) 0x07

#define ACK_POLICY_NO_REPLY (u_char) 0x00
#define ACK_POLICY_REPLY_TO_READ (u_char) 0x01
#define ACK_POLICY_REPLY_TO_ALL (u_char) 0x02

#define TORQUE_CONTROL_RAM_ADDR (u_char) 52
#define TORQUE_CONTROL_TORQUE_FREE (u_char) 0x00 // servo manually movable, operation command (I_JOG, S_JOG) not possible
#define TORQUE_CONTROL_BREAK_ON (u_char) 0x40 // servo stopped, operation command (I_JOG, S_JOG) not possible
#define TORQUE_CONTROL_TORQUE_ON (u_char) 0x60 // operation possible

#define LED_CONTROL_RAM_ADDR (u_char) 53
#define LED_CONTROL_OFF (u_char) 0x00
#define LED_CONTROL_GREEN (u_char) 0x01
#define LED_CONTROL_BLUE (u_char) 0x02
#define LED_CONTROL_RED (u_char) 0x04

#define STATUS_ERROR_RAM_ADDR (u_char) 48

#define MIN_POSITION_RAM_ADDR (u_char) 20
#define MIN_POSITION_EEP_ADDR (u_char) 26
#define MAX_POSITION_RAM_ADDR (u_char) 22
#define MAX_POSITION_EEP_ADDR (u_char) 28

#define ABSOLUTE_POSITION_RAM_ADDR (u_char) 60

#define SERVO_TYPE_UNKNOWN 0
#define SERVO_TYPE_DRS_0101 1
#define SERVO_TYPE_DRS_0602 2

#define DRS_0101_MIN_POS 0 // steps
#define DRS_0101_MAX_POS 1023 // steps
#define DRS_0101_RESOLUTION 0.325 // degrees/step
#define DRS_0101_ZERO_POS 512 // steps

#define DRS_0602_MIN_POS 10381 // steps
#define DRS_0602_MAX_POS  22129 // steps
#define DRS_0602_RESOLUTION 0.02778 // degrees/step
#define DRS_0602_ZERO_POS 16384 // steps

//typedef char u_char;

void calculateChecksum(u_char *checksum_1, u_char *checksum_2,
        u_char *packet_size,
        u_char servo_id, u_char command,
        std::vector<u_char> data)
{
	u_char checksum_tmp = 0;

	*packet_size = (u_char) 7 + (u_char) data.size();
	checksum_tmp = *packet_size ^ servo_id ^ command;
	for (int i = 0; i < data.size(); i++)
	{
		checksum_tmp = checksum_tmp ^ data[i];
	}
	*checksum_1 = (checksum_tmp) & (u_char) 0xFE;
	*checksum_2 = (~*checksum_1) & (u_char) 0xFE;
	//std::cout << "Checksum 1: " << (int) 0x30 << ", calculated: " << (int) checksum_1 << std::endl;
	//std::cout << "Checksum 2: " << (int) 0xCE << ", calculated: " << (int) checksum_2 << std::endl;
	return;
}

std::vector<u_char> makeCommandPacket(u_char servo_id,
					   u_char command,
                       std::vector<u_char> data)
{
   std::vector<u_char> command_packet;

   u_char checksum_1;
   u_char checksum_2;
   u_char packet_size;
   calculateChecksum(&checksum_1, &checksum_2, &packet_size, servo_id, command, data);

   command_packet.push_back(0xFF);
   command_packet.push_back(0xFF);
   command_packet.push_back(packet_size);
   command_packet.push_back(servo_id);
   command_packet.push_back(command);
   command_packet.push_back(checksum_1);
   command_packet.push_back(checksum_2);

   for (int i = 0; i < data.size(); i++) {
      command_packet.push_back(data[i]);
   }
   return command_packet;
}

/*std::vector<u_char> ramRead(u_char servo_id, std::vector<u_char> data) {
   std::vector<u_char> packet, tmp;
   packet = makeCommandPacket(servo_id, RAM_READ, data);

   my_serial->flushInput();
   my_serial->write(packet);
   packet.clear();
   my_serial->read(packet, 2);
   if (packet.size() < 2) {
      std::cout << "Not enough data received (" << packet.size() << " bytes received)" << std::endl;
   }
   else {
      if (packet[0] == 0xFF && packet[1] == 0xFF) {
         my_serial->read(tmp, 1);
         response_len = tmp[0];

         if (response_len > 0 && response_len < 15) {
            packet.insert(packet.end(), tmp.begin(), tmp.end());
            tmp.clear();
            my_serial->read(tmp, response_len - 3);
            packet.insert(packet.end(), tmp.begin(), tmp.end());
         } else {
            std::cout << "Can not get proper response" << std::endl;
         }
      }
   }
   return packet;
}*/

void ramWrite(u_char servo_id, std::vector<u_char> data)
{
   // RAM_WRITE command - 0x03
   std::vector<u_char> packet, tmp;
   packet = makeCommandPacket(servo_id, RAM_WRITE, data);

   //boost::asio::buffer commandBuffer(packet);

   printf("Command packet was :\n");
   for (int b=0; b<packet.size(); ++b)
	   printf("%2d - [0x%02X] %d\n", b, packet[b], packet[b]);

   printf("Writing command to port\n"); fflush(stdout);
   boost::asio::write(*port, boost::asio::buffer(packet));

   printf("Done.\n"); fflush(stdout);

   //std::vector<u_char> response(2);
   //boost::asio::read(*port, boost::asio::buffer(response, 2));

   //printf("Command response was [0x%02X, 0x%02X]\n", response[0], response[1]); fflush(stdout);

   /*my_serial->flushInput();
   my_serial->write(packet);
   packet.clear();
   my_serial->read(packet, 2);
   if (packet.size() < 2) {
      std::cout << "Not enugh data received (" << packet.size() << " bytes received)" << std::endl;
   } else {
      if (packet[0] == 0xFF && packet[1] == 0xFF) {
         //std::cout << "Header OK... ";
         my_serial->read(tmp, 1);
         response_len = tmp[0];
         if (response_len > 0 && response_len < 15) {
            packet.insert(packet.end(), tmp.begin(), tmp.end());
            tmp.clear();
            my_serial->read(tmp, response_len - 3);
            packet.insert(packet.end(), tmp.begin(), tmp.end());
         } else {
            std::cout << "Can not get proper response" << std::endl;
         }
      }
   }
   return;*/
}

std::vector<u_char> responseData(256);

void handler(
	const boost::system::error_code& error, // Result of operation.
	std::size_t bytes_transferred           // Number of bytes read.
)
{
	std::cout << "read complete : " << error << "bytes transferred : " << bytes_transferred;

	printf("Command response was :\n");
	for (int d=0; d<bytes_transferred; ++d)
		printf("%2d - [0x%02X] %d\n", d, responseData[d], responseData[d]);
}

/// Method to read EEP registers. returns len bytes starting at addr
std::vector<u_char> eepRead(u_char servo_id, u_char addr, u_char len)
{
	// EEP_READ command - 0x02
	std::vector<u_char> data;
	data.push_back(addr);
	data.push_back(len);
	std::vector<u_char> packet, tmp;
	packet = makeCommandPacket(servo_id, EEP_READ, data);

	printf("Command packet was :\n");
	for (int b=0; b<packet.size(); ++b)
		printf("%2d - [0x%02X] %d\n", b, packet[b], packet[b]);

	printf("Writing command to port\n"); fflush(stdout);
	boost::asio::write(*port, boost::asio::buffer(packet));

	printf("Done.\n"); fflush(stdout);

	// expect 11 + len bytes in response

	std::vector<u_char> response(11+len);
	boost::system::error_code ec;

	printf("about to read.\n"); fflush(stdout);

	int bytes_read = port->read_some(boost::asio::buffer(response, 11+len),ec);

	//port->async_read_some(boost::asio::buffer(responseData, 3), handler);

	printf("sent async read.\n"); fflush(stdout);

	for (int d=0; d<bytes_read; ++d)
		printf("%2d - [0x%02X] %d\n", d, response[d], response[d]);

	return response;
}

void setLed(u_char servoId, u_char ledState)
{
   std::vector<u_char> data;
   data.push_back(LED_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(ledState);
   ramWrite(servoId, data);
}

void setTorqueMode(u_char servoId, u_char torqueMode)
{
   std::vector<u_char> data;
   data.push_back(TORQUE_CONTROL_RAM_ADDR);
   data.push_back(0x01);
   data.push_back(torqueMode);
   ramWrite(servoId, data);
}

// create an S_JOG packet from the given servoId, time (seconds), angle(degrees) and led status mask
std::vector<u_char> makeSJOGPacket(u_char servoId, float time, float angle, u_char ledStatus)
{
	std::vector<u_char> packet;

	unsigned int timeOffset = time / 0.0112;
	if (timeOffset >= 256)
		printf("Warning: time beyond %f second limit!\n", 255 * 0.0112); fflush(stdout);
	packet.push_back(timeOffset & 0xff);

	unsigned int rawPosition = (angle / 0.02778) + 9903;
	packet.push_back(rawPosition & 0xff);
	packet.push_back((rawPosition >> 8) & 0xff);

	// create an options byte for a position jog command with VOR
	u_char options = 0x00 | ((ledStatus & 0x07) << 2);
	packet.push_back(options);

	packet.push_back(servoId);

	return packet;
}

void S_JOGCommand(u_char servoId, std::vector<u_char> packets)
{
	std::vector<u_char> packet = makeCommandPacket(servoId, S_JOG, packets);

	boost::asio::write(*port, boost::asio::buffer(packet));
}

void demoJOG(u_char servoId)
{
	std::vector<u_char> data;
	data.push_back(0x3C);
	data.push_back(0x00);
	data.push_back(0x02);
	data.push_back(0x04);
	data.push_back(servoId);

	std::vector<u_char> packet = makeCommandPacket(servoId, S_JOG, data);

	boost::asio::write(*port, boost::asio::buffer(packet));
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ Herkulex Test Node ]--");

	// setup ros for this node and get handle to ros system
	ros::init(argc, argv, "Herkulex_test");
	ros::start();

	std::string portName;
	ros::NodeHandle n("~");

	boost::asio::io_service io;

	n.param<std::string>("herkulex_port", portName, "/dev/ttyUSB0");

	ROS_INFO("connecting to Herkulex servo string on port \"%s\"", portName.c_str());
	try{
		port = new boost::asio::serial_port(io, portName);
		port->set_option(boost::asio::serial_port_base::baud_rate(115200));
	}
	catch (boost::system::system_error e)
	{
		ROS_ERROR("Failed to open port \"%s\" : %s", portName.c_str(), e.what());
	}

    ROS_INFO("--[ Herkulex Test Node: startup complete ]--");

    int servoId = 219;
    //for (int s=0; s<254; ++s)
    //{
    	printf("testing servo [%d]\n", servoId); fflush(stdout);
    	setLed(servoId, LED_CONTROL_BLUE);
    	eepRead(servoId, 0, 4);
    //}

    setTorqueMode(servoId, TORQUE_CONTROL_TORQUE_ON);

	boost::thread service_thread;
	service_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io ));

    ros::Rate loopRate(0.3);
    while(ros::ok())
    {

    	std::vector<u_char> jogPackets;
    	std::vector<u_char> tmp = makeSJOGPacket(219, 0.0, 45, LED_CONTROL_BLUE);
    	jogPackets.insert(jogPackets.end(), tmp.begin(), tmp.end());
    	tmp = makeSJOGPacket(219, 1.0, -45, LED_CONTROL_GREEN);
    	jogPackets.insert(jogPackets.end(), tmp.begin(), tmp.end());

    	S_JOGCommand(219, jogPackets);

    	//demoJOG(219);

    	ros::spinOnce();
    	loopRate.sleep();

    	ROS_WARN("Main Loop");
    }

	printf("--[ Herkulex Test Node: shutdown OK ]--\n");
	return 0;
}
