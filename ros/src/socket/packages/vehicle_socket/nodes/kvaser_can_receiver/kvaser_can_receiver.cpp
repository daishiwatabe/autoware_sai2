#include <ros/ros.h>
#include "kvaser_can.h"
#include <autoware_can_msgs/MicroBsuCanReceiver.h>

class kvaser_can_receiver
{
private:
	ros::NodeHandle nh_, private_nh_;
	KVASER_CAN kc;
public:
	kvaser_can_receiver(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	{
		kc.init(kvaser_channel);
	}

	bool isOpen() {return kc.isOpen();}

	void read_wait()
	{
		kc.read_wait(KVASER_CAN::READ_WAIT_INFINITE);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_can_receiver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	int kvaser_channel;
	private_nh.param("kvaser_channel", kvaser_channel, 0);
	kvaser_can_receiver kcr(nh, private_nh, kvaser_channel);
	if(kcr.isOpen() == false)
	{
		std::cerr << "error : open" << std::endl;
	}

	while(ros::ok())
	{
		kcr.read_wait();
		ros::spinOnce();
	}

	return 0;
}
