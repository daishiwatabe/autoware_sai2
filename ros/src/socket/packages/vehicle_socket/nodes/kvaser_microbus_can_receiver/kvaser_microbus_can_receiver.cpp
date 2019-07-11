#include <ros/ros.h>
#include <autoware_can_msgs/MicroBusCan.h>
#include "kvaser_can.h"

class kvaser_can_receiver
{
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Publisher pub_microbus_can_;

	KVASER_CAN kc;

	struct
	{
		bool read501, read502;
	} read_id_flag_;

public:
	kvaser_can_receiver(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	{
		read_id_flag_.read501 = read_id_flag_.read502 = false;
		kc.init(kvaser_channel);

		pub_microbus_can_ = nh_.advertise<autoware_can_msgs::MicroBusCan>("/microbus/can_receive", 10);
	}

	bool isOpen() {return kc.isOpen();}

	void read_wait()
	{
		canStatus res = kc.read_wait(100);
		if(res == canStatus::canOK)
		{
			switch(kc.get_id())
			{
			case 0x501:
			    {
				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan can;
					can.header.stamp = ros::Time::now();

					can.emergency = (data[7] == 0x55);
					unsigned char dmode = data[7] & 0x0F;
					can.drive_mode = (dmode == 0x0A);
					unsigned char smode = data[7] & 0xF0;
					can.steer_mode = (smode == 0xA0);

					unsigned char *vel_tmp = (unsigned char*)&can.velocity;
					vel_tmp[0] = data[3];  vel_tmp[1] = data[2];

					unsigned char *str_tmp = (unsigned char*)&can.steering_angle;
					str_tmp[0] = data[5];  str_tmp[1] = data[4];

					unsigned char *stroke_tmp = (unsigned char*)&can.pedal;
					//stroke_tmp[?] = data[?];  stroke_tmp[?] = data[?];

					can.read_counter = kc.get_read_counter();

					pub_microbus_can_.publish(can);
					read_id_flag_.read501 = true;
					break;
			    }
			case 0x502:
			    {
				    read_id_flag_.read502 = true;
					break;
			    }
			case 0x503:
			    {
				    read_id_flag_.read501 = read_id_flag_.read502 = false;
					break;
			    }
			}
		}
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
