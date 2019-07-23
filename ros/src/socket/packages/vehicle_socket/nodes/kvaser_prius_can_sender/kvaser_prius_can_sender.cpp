#include <ros/ros.h>
#include "kvaser_can.h"

class kvaser_can_sender
{
private:
	ros::NodeHandle nh_, private_nh_;

	KVASER_CAN kc;
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	{
		canStatus res = kc.init(kvaser_channel, canBITRATE_500K);
		if(res != canStatus::canOK) {std::cout << "open error" << std::endl;}
	}

	const bool isOpen() {return kc.isOpen();}

	void can_send()
	{

	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_prius_can_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	int kvaser_channel;
	bool use_position_checker;
	private_nh.param<int>("kvaser_channel", kvaser_channel, 0);
	kvaser_can_sender kcs(nh, private_nh, kvaser_channel);
	if(kcs.isOpen() == false)
	{
		std::cerr << "error : open" << std::endl;
	}

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		kcs.can_send();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
