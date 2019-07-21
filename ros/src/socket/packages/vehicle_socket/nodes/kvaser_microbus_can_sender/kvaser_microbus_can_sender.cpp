﻿#include <ros/ros.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/PositionChecker.h>
#include "kvaser_can.h"
#include <queue>

class kvaser_can_sender
{
private:
	//drive params
	const int PEDAL_MAX = 2000;
	const int PEDAL_MIN = -500;
	const double SPEED_LIMIT = 20;
	const int BRAKE_PEDAL_OFFSET = -50;

	//steer params
	const double HANDLE_MAX = 675;
	const double WHEEL_MAX = 36;
	const unsigned int STEER_VALUE_MARGIN = 20;

	double handle_angle_right_max = 660;
	double handle_angle_left_max = 670;
	double left_wheel_angle_right_max = 33;
	double right_wheel_angle_right_max = 38;
	double left_wheel_angle_left_max = 39.5;
	double right_wheel_angle_left_max = 33;
	double handle_actual_right_max = 13329;
	double handle_actual_left_max = 14177;
	double handle_offset = 188;

	//mode params
	const unsigned char MODE_STROKE   = 0x0A;
	const unsigned char MODE_VRLOCITY = 0x0B;

	//shift_param
	const static unsigned char SHIFT_P = 0;
	const static unsigned char SHIFT_R = 1;
	const static unsigned char SHIFT_N = 2;
	const static unsigned char SHIFT_D = 3;
	const static unsigned char SHIFT_2 = 4;
	const static unsigned char SHIFT_L = 5;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;

	ros::Publisher pub_microbus_can_sender_status_;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_;
	ros::Subscriber sub_microbus_can_501_, sub_microbus_can_502_;
	ros::Subscriber sub_emergency_reset_, sub_stroke_mode_, sub_velocity_mode_, sub_drive_mode_;
	ros::Subscriber sub_input_steer_flag_, sub_input_drive_flag_, sub_input_steer_value_, sub_input_drive_value_;
	ros::Subscriber sub_waypoint_param_, sub_position_checker_, sub_config_microbus_can_;
	ros::Subscriber sub_shift_auto_, sub_shift_position_, sub_emergency_stop_, sub_engine_start_, sub_ignition_;
	ros::Subscriber sub_wiper_, sub_light_high_, sub_light_low_, sub_light_small_;
	ros::Subscriber sub_horn_, sub_hazard_, sub_blinker_right_, sub_blinker_left_;

	KVASER_CAN kc;
	bool flag_drive_mode_, flag_steer_mode_;
	bool input_drive_mode_, input_steer_mode_;
	autoware_config_msgs::ConfigMicroBusCan setting_;
	unsigned char drive_control_mode_;
	autoware_can_msgs::MicroBusCan501 can_receive_501_;
	autoware_can_msgs::MicroBusCan502 can_receive_502_;
	geometry_msgs::TwistStamped twist_;
	short input_steer_, input_drive_;
	short pedal_;
	bool shift_auto_;
	unsigned char shift_position_;
	bool emergency_stop_, engine_start_, ignition_, wiper_;
	bool light_high_, light_low_, light_small_;
	bool horn_, hazard_, blinker_right_, blinker_left_;

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

	void callbackMicrobusCan501(const autoware_can_msgs::MicroBusCan501::ConstPtr &msg)
	{
		std::cout << "sub can" << std::endl;
		can_receive_501_ = *msg;
	}

	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
	{
		std::cout << "sub can" << std::endl;
		can_receive_502_ = *msg;
	}

	void callbackStrokeMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub StrokeMode" << std::endl;
		drive_control_mode_ = MODE_STROKE;
	}

	void callbackVelocityMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub VelocityMode" << std::endl;
		drive_control_mode_ = MODE_VRLOCITY;
	}

	void callbackShiftAuto(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string str = (msg->data == true) ? "shift_auto" : "shift_manual";
		std::cout << str << std::endl;
		shift_auto_ = msg->data;
	}

	void callbackShiftPosition(const std_msgs::UInt8::ConstPtr &msg)
	{
		std::cout << "shift position : " << (int)msg->data << std::endl;
		shift_position_ = msg->data;
	}

	void callbackConfigMicroBusCan(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		setting_ = *msg;
		publisStatus();
	}

	void callbackTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		std::cout << "sub twist" << std::endl;
		twist_ = *msg;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		pedal_ = msg->mb_pedal;
	}

	void callbackPositionChecker(const autoware_msgs::PositionChecker::ConstPtr &msg)
	{
		if(setting_.use_position_checker == true)
		{
			if(msg->stop_flag == true)
			{
				flag_drive_mode_ = false;
				flag_steer_mode_ = false;
				can_send();
			}
		}
	}

	void publisStatus()
	{
		autoware_can_msgs::MicroBusCanSenderStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.use_position_checker = setting_.use_position_checker;
		msg.use_input_steer = input_steer_mode_;
		msg.use_input_drive = input_drive_mode_;
		pub_microbus_can_sender_status_.publish(msg);
	}

	void callbackInputSteerFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		//input_steer_ = msg->data;
		input_steer_mode_ = msg->data;
		std::cout << "aaa" << std::endl;
		publisStatus();
	}

	void callbackInputSteerValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_steer_ = msg->data;
		std::cout << input_steer_ << std::endl;
	}

	void callbackInputDriveFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		//input_drive_ = msg->data;
		input_drive_mode_ = msg->data;
		std::cout << "ccc" << std::endl;
		publisStatus();
	}

	void callbackInputDriveValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_drive_ = msg->data;
		std::cout << input_drive_ << std::endl;
	}

	void callbackEmergencyStop(const std_msgs::Bool::ConstPtr &msg)
	{
		emergency_stop_ = msg->data;
		std::string str = (emergency_stop_) ? "true" : "false";
		std::cout << "emergency stop : " << str << std::endl;
	}

	void callbackEngineStart(const std_msgs::Bool::ConstPtr &msg)
	{
		engine_start_ = msg->data;
		std::string str = (engine_start_) ? "true" : "false";
		std::cout << "engine_start : " << str << std::endl;
	}

	void callbackIgnition(const std_msgs::Bool::ConstPtr &msg)
	{
		ignition_ = msg->data;
		std::string str = (ignition_) ? "true" : "false";
		std::cout << "Ignition : " << str << std::endl;
	}

	void callbackWiper(const std_msgs::Bool::ConstPtr &msg)
	{
		wiper_ = msg->data;
		std::string str = (wiper_) ? "true" : "false";
		std::cout << "wiper : " << str << std::endl;
	}

	void callbackLightHigh(const std_msgs::Bool::ConstPtr &msg)
	{
		light_high_ = msg->data;
		std::string str = (light_high_) ? "true" : "false";
		std::cout << "light_high : " << str << std::endl;
	}

	void callbackLightLow(const std_msgs::Bool::ConstPtr &msg)
	{
		light_low_ = msg->data;
		std::string str = (light_low_) ? "true" : "false";
		std::cout << "light_low : " << str << std::endl;
	}

	void callbackLightSmall(const std_msgs::Bool::ConstPtr &msg)
	{
		light_small_ = msg->data;
		std::string str = (light_small_) ? "true" : "false";
		std::cout << "light_small : " << str << std::endl;
	}

	void callbackHorn(const std_msgs::Bool::ConstPtr &msg)
	{
		horn_ = msg->data;
		std::string str = (horn_) ? "true" : "false";
		std::cout << "horn : " << str << std::endl;
	}

	void callbackHazard(const std_msgs::Bool::ConstPtr &msg)
	{
		hazard_ = msg->data;
		std::string str = (hazard_) ? "true" : "false";
		std::cout << "hazard : " << str << std::endl;
	}

	void callbackBlinkerRight(const std_msgs::Bool::ConstPtr &msg)
	{
		blinker_right_ = msg->data;
		std::string str = (blinker_right_) ? "true" : "false";
		std::cout << "blinker_right : " << str << std::endl;
	}

	void callbackBlinkerLeft(const std_msgs::Bool::ConstPtr &msg)
	{
		blinker_left_ = msg->data;
		std::string str = (blinker_left_) ? "true" : "false";
		std::cout << "blinker_left : " << str << std::endl;
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
		if(input_steer_mode_ == false)
		{
			/*double twist_ang = twist_.twist.angular.z*180.0 / M_PI;
			twist_ang *= HANDLE_MAX / WHEEL_MAX;
			twist_ang *= 20 * 1.0;
			steer_val = twist_ang;*/

			double twist_deg = twist_.twist.angular.z*180.0 / M_PI;
			if(twist_deg < 0)
			{
				double actual_max = handle_actual_right_max + handle_offset;
				double angle = (left_wheel_angle_right_max + right_wheel_angle_right_max) / 2.0;
				steer_val = twist_deg * actual_max / angle;
				steer_val *= 3;
			}
			else
			{
				double actual_max = handle_actual_left_max - handle_offset;
				double angle = (left_wheel_angle_left_max + right_wheel_angle_left_max) / 2.0;
				steer_val = twist_deg * actual_max / angle;
				steer_val *= 3;
			}
		}
		else steer_val = input_steer_;
		if(can_receive_501_.steer_auto == false) steer_val = 0;

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
			if(input_drive_mode_ == false)
			{
				double linearx = twist_.twist.linear.x;
				//if(linearx > 0.5 && linearx < 1.0) linearx = 1.0;
				double twist_drv = linearx *3.6 * 100; //std::cout << twist_drv << std::endl;
				drive_val = twist_drv;
			}
			else drive_val = input_drive_;
			if(can_receive_501_.drive_auto == false) drive_val = 0;

			unsigned char *drive_point = (unsigned char*)&drive_val;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
		else
		{
			unsigned char *drive_point = (unsigned char*)&pedal_;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
	}

	void bufset_car_control(unsigned char *buf)
	{
		buf[6] = buf[7] = 0;

		if(emergency_stop_ == true) buf[6] |= 0x80;
		else buf[6] |= 0x40;
		if(engine_start_ == true) buf[6] |= 0x20;
		if(ignition_ == true) buf[6] |= 0x10;
		if(wiper_ == true) buf[6] |= 0x08;
		if(light_high_ == true) buf[6] |= 0x04;
		if(light_low_ == true) buf[6] |= 0x02;
		if(light_small_ == true) buf[6] |= 0x01;
		if(horn_ == true) buf[7] |= 0x80;
		if(hazard_ == true) buf[7] |= 0x40;
		if(blinker_right_ == true) buf[7] |= 0x20;
		if(blinker_left_ == true) buf[7] |= 0x10;

		if (shift_auto_ == true)
		{
			buf[7] |= 0x08;
			switch (shift_position_)
			{
			case SHIFT_P:
				buf[7] |= 0x00;
				break;
			case SHIFT_R:
				buf[7] |= 0x01;
				break;
			case SHIFT_N:
				buf[7] |= 0x02;
				break;
			case SHIFT_D:
				buf[7] |= 0x03;
				break;
			case SHIFT_2:
				buf[7] |= 0x04;
				break;
			case SHIFT_L:
				buf[7] |= 0x05;
				break;
			}
		}
	}

	//HEV

	double estimate_accel = 0.0;
	int target_accel_level = 0;
	double accel_diff_sum = 0;
	double brake_diff_sum = 0;
	std::queue<double> accel_diff_buffer;
	std::queue<double> brake_diff_buffer;
	ros::Time proc_time;

	void clear_diff()
	{
		int i;

		accel_diff_sum = 0;
		brake_diff_sum = 0;

		for (i = 0; i < (int) accel_diff_buffer.size(); i++) {
			accel_diff_buffer.pop();
		}
		for (i = 0; i < (int) brake_diff_buffer.size(); i++) {
			brake_diff_buffer.pop();
		}
	}

	double _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
	{
		double e;
		static double e_prev = 0;
		double e_i;
		double e_d;
		double ret;

		// acclerate by releasing the brake pedal if pressed.
		if (can_receive_501_.pedal < BRAKE_PEDAL_OFFSET) {
			/*
		  double target_brake_stroke = vstate.brake_stroke - _BRAKE_RELEASE_STEP;
		  if (target_brake_stroke < 0)
			target_brake_stroke = 0;
		  ret = -target_brake_stroke; // if ret is negative, brake will be applied.
		  */

			// vstate has some delay until applying the current state.
			// perhaps we can just return 0 (release brake pedal) here to avoid acceleration delay.
			ret = 0;

		  /* reset PID variables. */
		  e_prev = 0;
		  //clear_diff();
		}
		else { // PID control
			double target_accel_stroke;

		  e = cmd_velocity - current_velocity;

		  e_d = e - e_prev;

		  accel_diff_sum += e;

      #if 0 // shouldn't we limit the cycles for I control?
		  accel_diff_buffer.push(e);
		  if (accel_diff_buffer.size() > _K_ACCEL_I_CYCLES) {
			double e_old = accel_diff_buffer.front();
			accel_diff_sum -= e_old;
			if (accel_diff_sum < 0) {
				accel_diff_sum = 0;
			}
			accel_diff_buffer.pop();
		  }
      #endif


		}
	}

	void StrokeControl(double current_velocity, double cmd_velocity)
	{
		static std::queue<double> vel_buffer;
		static uint vel_buffer_size = 10;
		double old_velocity = 0.0;

		// don't control if not in program mode.
		if (can_receive_501_.drive_mode == false) {
			clear_diff();
	  //#ifdef USE_BRAKE_LAMP
	  //    sndBrkLampOff();
	  //#endif /* USE_BRAKE_LAMP */
		  return;
		}

		ros::Time time = ros::Time::now();
		ros::Duration dura = time - proc_time;
		double cycle_time = dura.sec + dura.nsec*10E-9;//secondes

		if(proc_time.sec != 0 || proc_time.nsec != 0)
		{
			// estimate current acceleration.
			vel_buffer.push(current_velocity);
			if (vel_buffer.size() > vel_buffer_size) {
				old_velocity = vel_buffer.front();
			  vel_buffer.pop(); // remove old_velocity from the queue.
			  estimate_accel =
			    (current_velocity-old_velocity)/(cycle_time*vel_buffer_size);
			}

			std::cout << "estimate_accel: " << estimate_accel << std::endl;

			if (fabs(cmd_velocity) > current_velocity
			    && fabs(cmd_velocity) > 0.0
			    && current_velocity < SPEED_LIMIT) {
				double accel_stroke;
			  std::cout << "accelerate: current_velocity=" << current_velocity
			       << ", cmd_velocity=" << cmd_velocity << std::endl;
			  accel_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);
			  /*if (accel_stroke > 0) {
				cout << "ZMP_SET_DRV_STROKE(" << accel_stroke << ")" << endl;
				ZMP_SET_DRV_STROKE(accel_stroke);
				ZMP_SET_BRAKE_STROKE(0);
			  }
			  else {
				cout << "ZMP_SET_DRV_STROKE(0)" << endl;
				ZMP_SET_DRV_STROKE(0);
				cout << "ZMP_SET_BRAKE_STROKE(" << -accel_stroke << ")" << endl;
				ZMP_SET_BRAKE_STROKE(-accel_stroke);
			  }*/
			}
			/*else if (fabs(cmd_velocity) < current_velocity
					 && fabs(cmd_velocity) > 0.0) {
				double brake_stroke;
			  cout << "decelerate: current_velocity=" << current_velocity
				   << ", cmd_velocity=" << cmd_velocity << endl;
			  brake_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
			  if (brake_stroke > 0) {
				cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
				ZMP_SET_BRAKE_STROKE(brake_stroke);
				ZMP_SET_DRV_STROKE(0);
			  }
			  else {
				cout << "ZMP_SET_BRAKE_STROKE(0)" << endl;
				ZMP_SET_BRAKE_STROKE(0);
				cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
				ZMP_SET_DRV_STROKE(-brake_stroke);
			  }
			}*/
		}



		proc_time = time;
	}
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
	    : nh_(nh)
	    , private_nh_(p_nh)
	    , flag_drive_mode_(false)
	    , flag_steer_mode_(false)
	    , input_drive_mode_(false)
	    , input_steer_mode_(false)
	    , input_steer_(0)
	    , input_drive_(0)
	    , drive_control_mode_(MODE_STROKE)
	    , pedal_(0)
	    , proc_time(0)
	    , shift_auto_(false)
	    , shift_position_(0)
	    , emergency_stop_(false)
	    , engine_start_(false)
	    , ignition_(false)
	    , wiper_(false)
	    , light_high_(false)
	    , light_low_(false)
	    , light_small_(false)
	    , horn_(false)
	    , hazard_(false)
	    , blinker_right_(false)
	    , blinker_left_(false)
	{
		can_receive_501_.emergency = true;
		canStatus res = kc.init(kvaser_channel, canBITRATE_500K);
		if(res != canStatus::canOK) {std::cout << "open error" << std::endl;}

		setting_.use_position_checker == true;

		pub_microbus_can_sender_status_ = nh_.advertise<autoware_can_msgs::MicroBusCanSenderStatus>("/microbus/can_sender_status", 10, true);

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);
		sub_twist_cmd_ = nh_.subscribe("/twist_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		sub_microbus_can_501_ = nh_.subscribe("/microbus/can_receive501", 10, &kvaser_can_sender::callbackMicrobusCan501, this);
		sub_microbus_can_502_ = nh_.subscribe("/microbus/can_receive502", 10, &kvaser_can_sender::callbackMicrobusCan502, this);
		sub_emergency_reset_ = nh_.subscribe("/microbus/emergency_reset", 10, &kvaser_can_sender::callbackEmergencyReset, this);
		sub_input_steer_flag_ = nh_.subscribe("/microbus/input_steer_flag", 10, &kvaser_can_sender::callbackInputSteerFlag, this);
		sub_input_steer_value_ = nh_.subscribe("/microbus/input_steer_value", 10, &kvaser_can_sender::callbackInputSteerValue, this);
		sub_input_drive_flag_ = nh_.subscribe("/microbus/input_drive_flag", 10, &kvaser_can_sender::callbackInputDriveFlag, this);
		sub_input_drive_value_ = nh_.subscribe("/microbus/input_drive_value", 10, &kvaser_can_sender::callbackInputDriveValue, this);
		sub_stroke_mode_ = nh_.subscribe("/microbus/set_stroke_mode", 10, &kvaser_can_sender::callbackStrokeMode, this);
		sub_velocity_mode_ = nh_.subscribe("/microbus/set_velocity_mode", 10, &kvaser_can_sender::callbackVelocityMode, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &kvaser_can_sender::callbackWaypointParam, this);
		sub_position_checker_ = nh_.subscribe("/position_checker", 10, &kvaser_can_sender::callbackPositionChecker, this);
		sub_config_microbus_can_ = nh_.subscribe("/config/microbus_can", 10, &kvaser_can_sender::callbackConfigMicroBusCan, this);
		sub_shift_auto_ = nh_.subscribe("/microbus/shift_auto", 10, &kvaser_can_sender::callbackShiftAuto, this);
		sub_shift_position_ = nh_.subscribe("/microbus/shift_position", 10, &kvaser_can_sender::callbackShiftPosition, this);
		sub_emergency_stop_ = nh_.subscribe("/microbus/emergency_stop", 10, &kvaser_can_sender::callbackEmergencyStop, this);
		sub_engine_start_ = nh_.subscribe("/microbus/engine_start", 10, &kvaser_can_sender::callbackEngineStart, this);
		sub_ignition_ = nh_.subscribe("/microbus/ignition", 10, &kvaser_can_sender::callbackIgnition, this);
		sub_wiper_ = nh_.subscribe("/microbus/wiper", 10, &kvaser_can_sender::callbackWiper, this);
		sub_light_high_ = nh_.subscribe("/microbus/light_high", 10, &kvaser_can_sender::callbackLightHigh, this);
		sub_light_low_ = nh_.subscribe("/microbus/light_low", 10, &kvaser_can_sender::callbackLightLow, this);
		sub_light_small_ = nh_.subscribe("/microbus/light_small", 10, &kvaser_can_sender::callbackLightSmall, this);
		sub_horn_ = nh_.subscribe("/microbus/horn", 10, &kvaser_can_sender::callbackHorn, this);
		sub_hazard_ = nh_.subscribe("/microbus/hazard", 10, &kvaser_can_sender::callbackHazard, this);
		sub_blinker_right_ = nh_.subscribe("/microbus/blinker_right", 10, &kvaser_can_sender::callbackBlinkerRight, this);
		sub_blinker_left_ = nh_.subscribe("/microbus/blinker_left", 10, &kvaser_can_sender::callbackBlinkerLeft, this);

		publisStatus();

		proc_time.sec = 0;
		proc_time.nsec = 0;
	}

	const bool isOpen() {return kc.isOpen();}

	void can_send()
	{
		//if(can_receive_501_.emergency == false)
		{
			unsigned char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};

			bufset_mode(buf);
			bufset_steer(buf);
			bufset_drive(buf);
			bufset_car_control(buf);
			kc.write(0x100, (char*)buf, SEND_DATA_SIZE);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_microbus_can_sender");
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
