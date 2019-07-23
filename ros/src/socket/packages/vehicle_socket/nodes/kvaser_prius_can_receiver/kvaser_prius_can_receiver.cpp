#include <ros/ros.h>
#include "kvaser_can.h"
#include <bitset>

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
		kc.init(kvaser_channel, canBITRATE_1M);

		//pub_microbus_can_501_ = nh_.advertise<autoware_can_msgs::MicroBusCan501>("/microbus/can_receive501", 10);
		//pub_microbus_can_502_ = nh_.advertise<autoware_can_msgs::MicroBusCan502>("/microbus/can_receive502", 10);
	}

	bool isOpen() {return kc.isOpen();}

	void read_wait()
	{
		canStatus res = kc.read_wait(100);

		long id = kc.get_id();
		long id_high = id >> 6;
		long id_low = id & 0x3F;

		//std::cout << "high : " << std::hex << id_high << "   low : " << std::hex << id_low << std::endl;
		//std::cout << std::bitset<20>(id) << std::endl;
		if(id == 0x041)
		{
			kc.printReader();
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_microbus_can_receiver");
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
