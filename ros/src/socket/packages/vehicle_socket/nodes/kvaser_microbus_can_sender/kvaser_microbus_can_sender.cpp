#include <ros/ros.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_can_msgs/MicroBusCan.h>
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

	//mode params
	const unsigned char MODE_TORQUE   = 0x0A;
	const unsigned char MODE_VRLOCITY = 0x0B;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_, sub_microbus_can_;
	ros::Subscriber sub_emergency_reset_, sub_torque_mode_, sub_velocity_mode_, sub_drive_mode_;
	ros::Subscriber sub_auto_s_, sub_auto_d_, sub_auto_s_reset_, sub_auto_d_reset_;
	ros::Subscriber sub_waypoint_param_, sub_position_checker_, sub_config_microbus_can_;

	KVASER_CAN kc;
	bool flag_drive_mode_, flag_steer_mode_;
	bool auto_drive_mode_, auto_steer_mode_;
	autoware_config_msgs::ConfigMicroBusCan setting;
	unsigned char drive_control_mode_;
	autoware_can_msgs::MicroBusCan can_receive_;
	geometry_msgs::TwistStamped twist_;
	short auto_s_, auto_d_;
	short pedal_;

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

	void callbackMicrobusCan(const autoware_can_msgs::MicroBusCan::ConstPtr &msg)
	{
		std::cout << "sub can" << std::endl;
		can_receive_ = *msg;
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

	void callbackConfigMicroBusCan(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		setting = *msg;
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
		if(setting.use_position_checker == true)
		{
			if(msg->stop_flag == true)
			{
				flag_drive_mode_ = false;
				flag_steer_mode_ = false;
				can_send();
			}
		}
	}

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
			double twist_ang = twist_.twist.angular.z*180.0 / M_PI;
			twist_ang *= HANDLE_MAX / WHEEL_MAX;
			twist_ang *= 20 * 1.0;
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
				double linearx = twist_.twist.linear.x;
				//if(linearx > 0.5 && linearx < 1.0) linearx = 1.0;
				double twist_drv = linearx *3.6 * 100; //std::cout << twist_drv << std::endl;
				drive_val = twist_drv;
			}
			else drive_val = auto_d_;
			unsigned char *drive_point = (unsigned char*)&drive_val;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
		else
		{
			unsigned char *drive_point = (unsigned char*)&pedal_;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
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
		if (can_receive_.pedal < BRAKE_PEDAL_OFFSET) {
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
		if (can_receive_.drive_mode == false) {
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
	    , auto_drive_mode_(false)
	    , auto_steer_mode_(false)
	    , drive_control_mode_(MODE_TORQUE)
	    , pedal_(0)
	    , proc_time(0)
	{
		can_receive_.emergency = true;
		kc.init(kvaser_channel);

		setting.use_position_checker == true;

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);
		sub_twist_cmd_ = nh_.subscribe("/twist_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		sub_microbus_can_ = nh_.subscribe("/microbus/can_receive_", 10, &kvaser_can_sender::callbackMicrobusCan, this);
		sub_emergency_reset_ = nh_.subscribe("/microbus/emergency_reset", 10, &kvaser_can_sender::callbackEmergencyReset, this);
		sub_auto_s_ = nh_.subscribe("/microbus/auto_s", 10, &kvaser_can_sender::callbackAutoS, this);
		sub_auto_d_ = nh_.subscribe("/microbus/auto_d", 10, &kvaser_can_sender::callbackAutoD, this);
		sub_auto_s_reset_ = nh_.subscribe("/microbus/auto_s_reset", 10, &kvaser_can_sender::callbackAutoSReset, this);
		sub_auto_d_reset_ = nh_.subscribe("/microbus/auto_d_reset", 10, &kvaser_can_sender::callbackAutoDReset, this);
		sub_torque_mode_ = nh_.subscribe("/microbus/set_torque_mode", 10, &kvaser_can_sender::callbackTorqueMode, this);
		sub_velocity_mode_ = nh_.subscribe("/microbus/set_velocity_mode", 10, &kvaser_can_sender::callbackVelocityMode, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &kvaser_can_sender::callbackWaypointParam, this);
		sub_position_checker_ = nh_.subscribe("/position_checker", 10, &kvaser_can_sender::callbackPositionChecker, this);
		sub_config_microbus_can_ = nh_.subscribe("/config/microbus_can", 10, &kvaser_can_sender::callbackConfigMicroBusCan, this);

		proc_time.sec = 0;
		proc_time.nsec = 0;
	}

	const bool isOpen() {return kc.isOpen();}

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
