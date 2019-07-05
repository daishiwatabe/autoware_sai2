#include <ros/ros.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_can_msgs/MicroBusCan.h>
#include "kvaser_can.h"

class kvaser_can_sender
{
private:
	//drive params
	const int PEDAL_ACCEL_MAX = 2000;
	const int PEDAL_BRAKE_MAX = 2000;
	const double VELOCITY_MAX = 160;

	//steer params
	const double HANDLE_MAX = 675;
	const double WHEEL_MAX = 36;
	const unsigned int STEER_VALUE_MARGIN = 20;

	//mode params
	const unsigned char MODE_TORQUE   = 0x0A;
	const unsigned char MODE_VRLOCITY = 0x0B;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_, sub_microbus_can_;
	ros::Subscriber sub_emergency_reset_, sub_torque_mode_, sub_velocity_mode_, sub_drive_mode_;
	ros::Subscriber sub_auto_s_, sub_auto_d_, sub_auto_s_reset_, sub_auto_d_reset_;

	KVASER_CAN kc;
	bool flag_drive_mode_, flag_steer_mode_;
	bool auto_drive_mode_, auto_steer_mode_;
	unsigned char drive_control_mode_;
	autoware_can_msgs::MicroBusCan can_receive;
	geometry_msgs::TwistStamped twist;
	short auto_s_, auto_d_;

	void callbackAutoS(const std_msgs::Int16::ConstPtr &msg)
	{
		auto_s_ = msg->data;
		auto_steer_mode_ = true;
		std::cout << "aaa" << std::endl;
	}

	void callbackAutoD(const std_msgs::Int16::ConstPtr &msg)
	{
		auto_d_ = msg->data;
		auto_drive_mode_ = true;
		std::cout << "bbb" << std::endl;
	}

	void callbackAutoSReset(const std_msgs::Empty::ConstPtr &msg)
	{
		auto_steer_mode_ = false;
		std::cout << "ccc" << std::endl;
	}

	void callbackAutoDReset(const std_msgs::Empty::ConstPtr &msg)
	{
		auto_drive_mode_ = false;
		std::cout << "ddd" << std::endl;
	}

	void callbackEmergencyReset(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub Emergency" << std::endl;
		char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
		buf[0] = 0x55;
		kc.write(0x100, buf, SEND_DATA_SIZE);
		ros::Rate rate(1);
		rate.sleep();
		buf[0] = 0x00;
		kc.write(0x100, buf, SEND_DATA_SIZE);
	}

	void callbackTorqueMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub TorqueMode" << std::endl;
		drive_control_mode_ = MODE_TORQUE;
	}

	void callbackVelocityMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub VelocityMode" << std::endl;
		drive_control_mode_ = MODE_VRLOCITY;
	}

	void callbackDModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub DMode : " << flag << std::endl;
		flag_drive_mode_ = msg->data;
	}

	void callbackSModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub SMode : " << flag << std::endl;
		flag_steer_mode_ = msg->data;
	}

	void callbackMicrobusCan(const autoware_can_msgs::MicroBusCan::ConstPtr &msg)
	{
		std::cout << "sub can" << std::endl;
		can_receive = *msg;
	}

	void callbackTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		std::cout << "sub twist" << std::endl;
		twist = *msg;
	}

	void bufset_mode(unsigned char *buf)
	{
		unsigned char mode = 0;
		if(flag_drive_mode_ == true) mode |= drive_control_mode_;
		if(flag_steer_mode_ == true) mode |= 0xA0;
		buf[0] = mode;  buf[1] = 0;
	}

	void bufset_steer(unsigned char *buf)
	{
		short steer_val;
		if(auto_steer_mode_ == false)
		{
			double twist_ang = twist.twist.angular.z*180.0 / M_PI;
			twist_ang *= HANDLE_MAX / WHEEL_MAX;
			twist_ang *= -20 * 1.1;
			steer_val = twist_ang;
		}
		else steer_val = auto_s_;
		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];
	}

	void bufset_drive(unsigned char *buf)
	{
		/*double twist_drv = twist.twist.linear.x;
		short drive_val = (twist_drv / PEDAL_ACCEL_MAX) * VELOCITY_MAX;
		unsigned char *drive_point = (unsigned char*)&drive_val;
		buf[4] = drive_point[1];  buf[5] = drive_point[0];*/

		//std::cout << "mode : " << (int)drive_control_mode_ << "   vel : " << (int)MODE_VRLOCITY << std::endl;
		if(drive_control_mode_ == MODE_VRLOCITY)
		{
			short drive_val;
			if(auto_drive_mode_ == false)
			{
				double twist_drv = twist.twist.linear.x *3.6 * 100; //std::cout << twist_drv << std::endl;
				drive_val = twist_drv/2;
			}
			else drive_val = auto_d_;
			unsigned char *drive_point = (unsigned char*)&drive_val;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
		else
		{

		}
	}
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	    , flag_drive_mode_(false)
	    , flag_steer_mode_(false)
	    , auto_drive_mode_(false)
	    , auto_steer_mode_(false)
	    , drive_control_mode_(MODE_TORQUE)
	{
		can_receive.emergency = true;
		kc.init(kvaser_channel);

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);
		sub_twist_cmd_ = nh_.subscribe("/twist_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		sub_microbus_can_ = nh_.subscribe("/microbus/can_receive", 10, &kvaser_can_sender::callbackMicrobusCan, this);
		sub_emergency_reset_ = nh_.subscribe("/microbus/emergency_reset", 10, &kvaser_can_sender::callbackEmergencyReset, this);
		sub_auto_s_ = nh_.subscribe("/microbus/auto_s", 10, &kvaser_can_sender::callbackAutoS, this);
		sub_auto_d_ = nh_.subscribe("/microbus/auto_d", 10, &kvaser_can_sender::callbackAutoD, this);
		sub_auto_s_reset_ = nh_.subscribe("/microbus/auto_s_reset", 10, &kvaser_can_sender::callbackAutoSReset, this);
		sub_auto_d_reset_ = nh_.subscribe("/microbus/auto_d_reset", 10, &kvaser_can_sender::callbackAutoDReset, this);
	}

	bool isOpen() {return kc.isOpen();}

	void can_send()
	{
		unsigned char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};

		bufset_mode(buf);
		bufset_steer(buf);
		bufset_drive(buf);
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

	//kcs.emergency_reset();
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		kcs.can_send();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
