#include <ros/ros.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include "kvaser_can.h"

class kvaser_can_receiver
{
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Publisher pub_microbus_can_501_, pub_microbus_can_502_, pub_microbus_can_503_;

	//liesse params
	double handle_angle_right_max = 730;
	double handle_angle_left_max = 765;
	double wheelrad_to_steering_can_value_left = 20691.8161699557;//20952.8189547718;
	double wheelrad_to_steering_can_value_right = 20802.5331916036;//20961.415734248;
	//double wheelrad_to_steering_can_value_left = 20691.8161699557;
	//double wheelrad_to_steering_can_value_right = 20802.5331916036;
	double angle_magn_right = handle_angle_right_max / 15000;
	double angle_magn_left = handle_angle_left_max / 15000;

	KVASER_CAN kc;

	struct
	{
		bool read501, read502;
	} read_id_flag_;

	std::vector<short> velocity_list_;
	std::vector<short> acceleration_list_;
	std::vector<short> angle_list_;
	ros::Time prev_time_502_;
	const int list_pushback_size = 5;
public:
	kvaser_can_receiver(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	{
		prev_time_502_ = ros::Time::now();
		read_id_flag_.read501 = read_id_flag_.read502 = false;
		kc.init(kvaser_channel, canBITRATE_500K);

		pub_microbus_can_501_ = nh_.advertise<autoware_can_msgs::MicroBusCan501>("/microbus/can_receive501", 10);
		pub_microbus_can_502_ = nh_.advertise<autoware_can_msgs::MicroBusCan502>("/microbus/can_receive502", 10);
		pub_microbus_can_503_ = nh_.advertise<autoware_can_msgs::MicroBusCan503>("/microbus/can_receive503", 10);
	}

	bool isOpen() {return kc.isOpen();}

	void read_wait()
	{
		canStatus res = kc.read_wait(100);
		if(kc.get_id() == 0x501 || kc.get_id() == 0x502 || kc.get_id() == 0x503) kc.printReader();

		if(res == canStatus::canOK)
		{
			switch(kc.get_id())
			{
			case 0x501:
			    {
				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan501 can;
					can.header.stamp = ros::Time::now();

					can.emergency = (data[0] == 0x55);
					unsigned char dmode0 = data[0] & 0x0F;
					unsigned char dmode1 = data[1] & 0x0F;
					//can.drive_auto = (dmode1==0x10 || dmode1==0x11) ? 0xA : 0;
					/*switch(dmode1)
					{
					case 0x0A:
						can.drive_auto = true;
						break;
					case 0x0B:
						can.drive_auto = true;
						break;
					default:
						can.drive_auto = false;
					}*/
					switch(dmode1)
					{
					case autoware_can_msgs::MicroBusCan501::STEER_AUTO:
						can.drive_auto = autoware_can_msgs::MicroBusCan501::STEER_AUTO;
						can.drive_mode = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE;
						break;
					case autoware_can_msgs::MicroBusCan501::STEER_AUTO+1:
						can.drive_auto = autoware_can_msgs::MicroBusCan501::STEER_AUTO;
						can.drive_mode = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY;
						break;
					default:
						can.drive_auto = 0;
						can.drive_mode = 0;
						break;
					}
					//can.drive_auto = (dmode == 0x0A);
					unsigned char smode = data[1] & 0xF0;
					can.steer_auto = smode >> 4;

					unsigned char *vel_tmp = (unsigned char*)&can.velocity;
					vel_tmp[0] = data[5];  vel_tmp[1] = data[4];

					unsigned char *str_tmp = (unsigned char*)&can.steering_angle;
					str_tmp[0] = data[3];  str_tmp[1] = data[2];

					unsigned char *stroke_tmp = (unsigned char*)&can.pedal;
					//stroke_tmp[?] = data[?];  stroke_tmp[?] = data[?];

					can.read_counter = kc.get_read_counter();

					if(data[6] & 0x80 != 0) can.emergency_stop = 2;
					else if(data[6] & 0x40 != 0) can.emergency_stop = 1;
					else can.emergency_stop = 0;
					if(data[6] & 0x20 != 0) can.side_brake = 2;
					else if(data[6] & 0x10 != 0) can.side_brake = 1;
					else can.side_brake = 0;
					if(data[6] & 0x08 != 0) can.automatic_door = 2;
					else if(data[6] & 0x04 != 0) can.automatic_door = 1;
					else can.automatic_door = 0;
					can.blinker_right = (data[6] & 0x02 != 0) ? true : false;
					can.blinker_left = (data[6] & 0x01 != 0) ? true : false;
					/*if(data[6] & 0x80 != 0) can.emergency_stop = 2;
					else if(data[6] & 0x40 != 0) can.emergency_stop = 1;
					else can.emergency_stop = 0;
					can.engine_start = (data[6] & 0x20 != 0) ? true : false;
					can.ignition = (data[6] & 0x10 != 0) ? true : false;
					can.wiper = (data[6] & 0x08 != 0) ? true : false;
					can.light_high = (data[6] & 0x04 != 0) ? true : false;
					can.light_low = (data[6] & 0x02 != 0) ? true : false;
					can.light_small = (data[6] & 0x01 != 0) ? true : false;
					can.horn = (data[7] & 0x80 != 0) ? true : false;
					can.hazard = (data[7] & 0x40 != 0) ? true : false;
					can.blinker_right = (data[7] & 0x20 != 0) ? true : false;
					can.blinker_left = (data[7] & 0x10 != 0) ? true : false;
					can.shift = data[7] & 0x0F;*/

					pub_microbus_can_501_.publish(can);
					read_id_flag_.read501 = true;
					break;
			    }
			case 0x502:
			    {
				    ros::Time nowtime = ros::Time::now();

				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan502 can;
					can.header.stamp = nowtime;

					unsigned char *vel_tmp = (unsigned char*)&can.velocity_actual;
					vel_tmp[0] = data[7];  vel_tmp[1] = data[6];

					short acceleration = 0;
					can.cycle_time = nowtime.toSec() - prev_time_502_.toSec();
					if(velocity_list_.size() != 0)
					{
						short acc = (velocity_list_[0] - can.velocity_actual) / can.cycle_time;
						can.acceleration_actual = acc;
						acceleration_list_.insert(acceleration_list_.begin(), acc);
						if(acceleration_list_.size() > list_pushback_size) acceleration_list_.resize(list_pushback_size);
						int sum = 0;
						for(int i=0; i<acceleration_list_.size(); i++) sum += acceleration_list_[i];
						can.acceleration_average = sum / acceleration_list_.size();

						std::vector<short> acceleration_med(acceleration_list_.size());
						std::copy(acceleration_list_.begin(), acceleration_list_.end(), acceleration_med.begin());
						std::sort(acceleration_med.begin(), acceleration_med.end());
						can.acceleration_median = acceleration_med[(int)std::round(acceleration_med.size()/(double)2.0)];
					}


					velocity_list_.insert(velocity_list_.begin(), can.velocity_actual);
					if(velocity_list_.size() > list_pushback_size) velocity_list_.resize(list_pushback_size);
					int sum = 0;
					for(int i=0; i<velocity_list_.size(); i++) sum += velocity_list_[i];
					can.velocity_average = sum / velocity_list_.size();

					std::vector<short> velocity_med(velocity_list_.size());
					std::copy(velocity_list_.begin(), velocity_list_.end(), velocity_med.begin());
					std::sort(velocity_med.begin(), velocity_med.end());
					can.velocity_median = velocity_med[(int)std::round(velocity_med.size()/(double)2.0)];

					can.velocity_mps = (double)can.velocity_actual / (100.0);

					unsigned char *str_tmp = (unsigned char*)&can.angle_actual;
					str_tmp[0] = data[5];  str_tmp[1] = data[4];
					angle_list_.insert(angle_list_.begin(), can.angle_actual);
					if(angle_list_.size() == list_pushback_size) angle_list_.resize(list_pushback_size);
					if(can.velocity_actual >= 0) can.angle_deg = can.angle_actual * angle_magn_left;
					else can.angle_deg = can.angle_actual * angle_magn_right;

					unsigned char status0 = data[0],  status1 = data[1];
					can.clutch = (status1 & 0x40) ? false : true;

					can.read_counter = kc.get_read_counter();

					pub_microbus_can_502_.publish(can);
				    read_id_flag_.read502 = true;
					prev_time_502_ = nowtime;
					break;
			    }
			case 0x503:
			    {
				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan503 can;
					can.header.stamp = ros::Time::now();

					unsigned char *voltage_tmp = (unsigned char*)&can.pedal_voltage;
					voltage_tmp[0] = data[3];  voltage_tmp[1] = data[2];

					unsigned char *displacement_tmp = (unsigned char*)&can.pedal_displacement;
					displacement_tmp[0] = data[5];  displacement_tmp[1] = data[4];

					unsigned char *engine_rotation_tmp = (unsigned char*)&can.engine_rotation;
					engine_rotation_tmp[0] = data[7];  engine_rotation_tmp[1] = data[6];

					unsigned char status0 = data[0],  status1 = data[1];
					can.clutch = (status1 & 0x40) ? false : true;

					can.read_counter = kc.get_read_counter();

					pub_microbus_can_503_.publish(can);
					std::cout << "pedal_voltage : " << can.pedal_voltage << std::endl;
					std::cout << "pedal_displacement : " << can.pedal_displacement << std::endl;
					std::cout << "engine_rotation : " << can.engine_rotation;
				    read_id_flag_.read501 = read_id_flag_.read502 = false;
					break;
			    }
			}
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_prius_can_receiver");
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
