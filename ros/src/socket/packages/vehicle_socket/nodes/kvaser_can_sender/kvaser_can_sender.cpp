#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_can_msgs/MicroBusCan.h>
#include "kvaser_can.h"

class kvaser_can_sender
{
private:
	ros::NodeHandle nh_, private_nh_;
	KVASER_CAN kc;

	ros::Subscriber sub_twist_cmd_;
	ros::Subscriber sub_micro_bus_can_;

	void callbackTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		std::cout << "linear x : " << msg->twist.linear.x << std::endl;
		std::cout << "angular z : " << msg->twist.angular.z << std::endl;

		/*int vel_int = (int)floor(msg->twist.linear.x);
		int ang_int = (int)floor(msg->twist.angular.z);
		int vel_dec = (int)((msg->twist.linear.x - vel_int) *100);
		int ang_dec = (int)((msg->twist.angular.z - ang_int) *100);
		std::cout << "linear x : int " << vel_int << " : dec " << vel_dec << std::endl;
		std::cout << "angular z : int " << ang_int << " : dec " << ang_dec << std::endl;

		unsigned char buf[8] = {0,0,0,0,0,0,0,0xAA};
		//buf[2] = vel_int;  buf[3] = vel_dec;
		//buf[6] = ang_int;  buf[7] = ang_dec;
		kc.write(0x100, (char*)buf, 8);*/

		char buf[8] = {0,0,0,0,0,0,0,0};
		buf[0] = 0xA0;
		short tmp = msg->twist.angular.z * 25000;
		char *s_steer = (char*)&tmp;
		buf[2] = s_steer[1];  buf[3] = s_steer[0];
		kc.write(0x100, buf, 8);
	}

	void callbackMicroBusCan(const autoware_can_msgs::MicroBusCan::ConstPtr &msg)
	{
		char buf[8] = {0,0,0,0,0,0,0,0};
		//short *shortbuf = (short*)buf;
		//unsigned short *ushortbuf = (unsigned short*)buf;

		/*ushortbuf[0] = msg->switch_command;
		shortbuf[1] = msg->steering_angle;
		shortbuf[2] = msg->velocity;
		ushortbuf[3] = msg->switch_command;*/
		if(msg->steer_mode==true)
		{
			if(msg->drive_mode==true) buf[0] = 0xAA;
			else buf[0] = 0xA0;
		}
		else
		{
			if(msg->drive_mode==true) buf[0] = 0x0A;
			else buf[0] = 0x00;
		}
		buf[1] = 0;

		char *s_steer = (char*)&msg->steering_angle;
		buf[2] = s_steer[1];  buf[3] = s_steer[0];

		char *s_drive = (char*)&msg->velocity;
		buf[4] = s_drive[1];  buf[5] = s_drive[0];

		buf[6] = buf[7] = 0;
		kc.write(0x100, buf, 8);

	}
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	{
		kc.init(kvaser_channel);

		//char buf[8] = {0x1, 0x2, 0x3, 0x4,0x5, 0x6, 0x7, 0x8 };
		//kc.write(0x100, buf, 8);

		sub_twist_cmd_ = nh_.subscribe("twist_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		//sub_micro_bus_can_ = nh_.subscribe("micro_bus_can", 10, &kvaser_can_sender::callbackMicroBusCan, this);
		/*unsigned char buf[8] = {0xAA,0,0xFE,0x0,0,0,0,0};
		//buf[2] = vel_int;  buf[3] = vel_dec;
		//buf[6] = ang_int;  buf[7] = ang_dec;
		kc.write(0x100, (char*)buf, 8);*/
	}

	bool isOpen() {return kc.isOpen();}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_can_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	int kvaser_channel;
	private_nh.param("kvaser_channel", kvaser_channel, 0);
	kvaser_can_sender kcs(nh, private_nh, kvaser_channel);
	if(kcs.isOpen() == false)
	{
		std::cerr << "error : open" << std::endl;
	}

	while(ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}
