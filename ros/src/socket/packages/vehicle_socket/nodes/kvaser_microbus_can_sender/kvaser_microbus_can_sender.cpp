#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_can_msgs/MicroBusCan.h>
#include "kvaser_can.h"

class kvaser_can_sender
{
private:
	const int HANDLE_RIGHT_MAX = 2000;
	const int HANDLE_LEFT_MAX  = 2000;
	const double WHEEL_RIGHT_MAX  = 40;
	const double WHEEL_LEFT_MAX   = 40;
	const unsigned int SEND_DATA_SIZE = 8;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_, sub_microbus_can_;
	ros::Subscriber sub_emergency_reset_;

	KVASER_CAN kc;
	bool drive_mode_, steer_mode_;
	autoware_can_msgs::MicroBusCan can_receive;
	geometry_msgs::TwistStamped twist;

	void callbackEmergencyReset(const std_msgs::Empty::ConstPtr &msg)
	{
		char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
		buf[7] = 0x55;
		kc.write(0x100, buf, SEND_DATA_SIZE);
		ros::Rate rate(1);
		rate.sleep();
		buf[7] = 0x00;
		kc.write(0x100, buf, SEND_DATA_SIZE);
	}

	void callbackDModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		drive_mode_ = msg->data;
	}

	void callbackSModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		steer_mode_ = msg->data;
	}

	void callbackMicrobusCan(const autoware_can_msgs::MicroBusCan::ConstPtr &msg)
	{
		can_receive = *msg;
	}

	void callbackTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		twist = *msg;
	}
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	    , drive_mode_(false)
	    , steer_mode_(false)
	{
		can_receive.emergency = true;
		kc.init(kvaser_channel);

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);
		sub_twist_cmd_ = nh_.subscribe("/twist_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		sub_microbus_can_ = nh_.subscribe("/microbus/can_receive", 10, &kvaser_can_sender::callbackMicrobusCan, this);
		sub_emergency_reset_ = nh_.subscribe("/microbus/emergency_reset", 10, &kvaser_can_sender::callbackEmergencyReset, this);
	}

	bool isOpen() {return kc.isOpen();}

	void can_send()
	{
		unsigned char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};

		unsigned char mode = 0;
		if(drive_mode_ == true) mode |= 0x0A;
		if(steer_mode_ == true) mode |= 0xA0;
		buf[0] = mode;  buf[1] = 0;

		double twist_ang = twist.twist.angular.z*180.0/M_PI;
		short steer_val;
		if(twist_ang >= 0)
			steer_val = (twist_ang / WHEEL_RIGHT_MAX) * HANDLE_RIGHT_MAX;
		else
			steer_val = (twist_ang / WHEEL_LEFT_MAX) * HANDLE_LEFT_MAX;
		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];

		double twist_drv = twist.twist.linear.x;

		kc.write(0x100, (char*)buf, SEND_DATA_SIZE);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_microbus_can_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	int kvaser_channel;
	private_nh.param("kvaser_channel", kvaser_channel, 0);
	kvaser_can_sender kcs(nh, private_nh, kvaser_channel);
	if(kcs.isOpen() == false)
	{
		std::cerr << "error : open" << std::endl;
	}

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		kcs.can_send();
		loop_rate.sleep();
	}
	return 0;
}
