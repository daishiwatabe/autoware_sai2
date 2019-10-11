#include <ros/ros.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_config_msgs/ConfigVelocitySet.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/PositionChecker.h>
#include <autoware_msgs/WaypointParam.h>
#include "kvaser_can.h"
#include <time.h>

enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4,
};

class waypoint_param_geter
{
public:
	ros::Time pause_time_;
	short pause_speed_;
	char automatic_door_;
	bool automatic_door_taiki_;

	waypoint_param_geter()
	{
		pause_time_ = ros::Time::now();
		pause_speed_ = 0;
		automatic_door_ = 0;
		automatic_door_taiki_ = false;
	}
};

struct LIMIT_ANGLE_FROM_VELOCITY_STRUCT
{
	double velocity;
	double limit_angle_top;
	double limit_angle_bottom;
};

class LIMIT_ANGLE_FROM_VELOCITY_CLASS
{
private:
	std::vector<LIMIT_ANGLE_FROM_VELOCITY_STRUCT> lafvs;
public:
	LIMIT_ANGLE_FROM_VELOCITY_CLASS()
	{
		lafvs.clear();
		LIMIT_ANGLE_FROM_VELOCITY_STRUCT data={0,700,-700};
		lafvs.push_back(data);
	}

	void add(LIMIT_ANGLE_FROM_VELOCITY_STRUCT data) {lafvs.push_back(data);}

	const LIMIT_ANGLE_FROM_VELOCITY_STRUCT getLimit(double vel)
	{
		for(int i=0;i<lafvs.size()-1;i++)
		{
			double bottom = lafvs[i].velocity;
			double top = lafvs[i+1].velocity;
			std::cout << "velocity_range : " << top << "," << vel << "," << bottom << std::endl;
			if(vel >= bottom && vel < top)
			{
				return lafvs[i+1];
			}
		}
//std::cout << "aaaaaaaaaaaaa\n";
		return lafvs[lafvs.size()-1];
	}
};

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

	//vanet params
	//double handle_angle_right_max = 660;
	//double handle_angle_left_max = 670;
	double left_wheel_angle_right_max = 33;
	double right_wheel_angle_right_max = 38;
	double left_wheel_angle_left_max = 39.5;
	double right_wheel_angle_left_max = 33;
	double handle_actual_right_max = 13329;
	double handle_actual_left_max = 14177;
	double handle_offset = 188;

	//liesse params
	double handle_angle_right_max = 730;
	double handle_angle_left_max = 765;
	double wheelrad_to_steering_can_value_left = 20691.8161699557;//20952.8189547718;
	double wheelrad_to_steering_can_value_right = 20802.5331916036;//20961.415734248;
	//double wheelrad_to_steering_can_value_left = 20691.8161699557;
	//double wheelrad_to_steering_can_value_right = 20802.5331916036;
	double angle_magn_right = wheelrad_to_steering_can_value_right / handle_angle_right_max;
	double angle_magn_left = wheelrad_to_steering_can_value_left / handle_angle_left_max;

	//mode params
	const unsigned char MODE_STROKE   = 0x0A;
	const unsigned char MODE_VERLOCITY = 0x0B;

	//shift_param
	const static unsigned char SHIFT_P = 0;
	const static unsigned char SHIFT_R = 1;
	const static unsigned char SHIFT_N = 2;
	const static unsigned char SHIFT_D = 3;
	const static unsigned char SHIFT_2 = 4;
	const static unsigned char SHIFT_L = 5;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;

	//safety
	const LIMIT_ANGLE_FROM_VELOCITY_STRUCT limit10 = {10,680,-680}, limit15 = {15,360,-360}, limit20 = {20,180,-180}, limit30={30,90,-90}, limit40={40,45,-45};
	LIMIT_ANGLE_FROM_VELOCITY_CLASS lafvc;
	bool dengerStopFlag = false;//自動運転が失敗しそうな場に止めるフラグ

	ros::Publisher pub_microbus_can_sender_status_;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_;
	ros::Subscriber sub_microbus_can_501_, sub_microbus_can_502_, sub_microbus_can_503_;
	ros::Subscriber sub_emergency_reset_, sub_stroke_mode_, sub_velocity_mode_, sub_drive_control_;
	ros::Subscriber sub_input_steer_flag_, sub_input_drive_flag_, sub_input_steer_value_, sub_input_drive_value_;
	ros::Subscriber sub_waypoint_param_, sub_position_checker_, sub_config_microbus_can_;
	ros::Subscriber sub_shift_auto_, sub_shift_position_;
	ros::Subscriber sub_emergency_stop_, sub_engine_start_, sub_ignition_;
	ros::Subscriber sub_wiper_, sub_light_high_, sub_light_low_, sub_light_small_;
	ros::Subscriber sub_horn_, sub_hazard_, sub_blinker_right_, sub_blinker_left_, sub_blinker_stop_;
	ros::Subscriber sub_automatic_door_, sub_drive_gasu_breake_, sub_steer_gasu_breake_;
	ros::Subscriber sub_econtrol_, sub_obtracle_waypoint_;

	KVASER_CAN kc;
	bool flag_drive_mode_, flag_steer_mode_;
	bool input_drive_mode_, input_steer_mode_;
	autoware_config_msgs::ConfigMicroBusCan setting_;
	unsigned char drive_control_mode_;
	autoware_can_msgs::MicroBusCan501 can_receive_501_;
	autoware_can_msgs::MicroBusCan502 can_receive_502_;
	autoware_can_msgs::MicroBusCan503 can_receive_503_;
	autoware_msgs::VehicleCmd twist_;
	short input_steer_, input_drive_;
	short pedal_;
	bool shift_auto_;
	unsigned char shift_position_, drive_gasu_breake_, steer_gasu_breake_, automatic_door_;
	unsigned char emergency_stop_;
	bool engine_start_, ignition_, wiper_;
	bool light_high_, light_low_, light_small_, horn_;
	bool hazard_, blinker_right_, blinker_left_, blinker_stop_;
	EControl econtrol;
	int obstracle_waypoint_;
	autoware_msgs::WaypointParam waypoint_param_;

	ros::Time automatic_door_time_;
	ros::Time blinker_right_time_, blinker_left_time_, blinker_stop_time_;

	waypoint_param_geter wpg_;

	void callbackObstracleWaypoint(const std_msgs::Int32::ConstPtr &msg)
	{
		obstracle_waypoint_ = msg->data;
	}

	void callbackEControl(const std_msgs::Int8::ConstPtr &msg)
	{
		econtrol = (EControl)msg->data;
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
		std::cout << "sub can_501" << std::endl;
		can_receive_501_ = *msg;
		//if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
		//	drive_control_mode_ = MODE_STROKE;
	}

	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
	{
		std::cout << "sub can_502" << std::endl;
		if(msg->clutch==true && can_receive_502_.clutch==false)
		{
			input_steer_mode_ = false; std::cout << "aaa" << std::endl;
			std::string safety_error_message = "";
			publisStatus(safety_error_message);
		}
		if(msg->clutch==false && can_receive_502_.clutch==true)
		{
			input_steer_mode_ = true;
			std::string safety_error_message = "";
			publisStatus(safety_error_message);
		}

		can_receive_502_ = *msg;
		brake_mode_changer();
	}

	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
	{
		std::cout << "sub can_503" << std::endl;
		if(msg->clutch==true && can_receive_503_.clutch==false)
		{
			drive_control_mode_ = MODE_VERLOCITY;
			shift_auto_ = true;
			input_drive_mode_ = false;
			std::string safety_error_message = "";
			publisStatus(safety_error_message);
		}
		if(msg->clutch==false && can_receive_503_.clutch==true)
		{
			drive_control_mode_ = MODE_STROKE;
			shift_auto_ = false;
			input_drive_mode_ = true;
			std::string safety_error_message = "";
			publisStatus(safety_error_message);
		}
		can_receive_503_ = *msg;
	}

	void callbackStrokeMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub StrokeMode" << std::endl;
		drive_control_mode_ = MODE_STROKE;
	}

	void callbackVelocityMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub VelocityMode" << std::endl;
		drive_control_mode_ = MODE_VERLOCITY;
	}

	void callbackDriveControl(const std_msgs::Int8::ConstPtr &msg)
	{
		if(msg->data == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY)
		{
			std::cout << "sub VelocityMode" << std::endl;
			drive_control_mode_ = MODE_VERLOCITY;
		}
		else if(msg->data == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE)
		{
			std::cout << "sub StrokeMode" << std::endl;
			drive_control_mode_ = MODE_STROKE;
		}
		else std::cout << "Control mode flag error" << std::endl;
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

	void automaticDoorSet(unsigned char flag)
	{
		automatic_door_ = flag;

		ros::Time nowtime = ros::Time::now();
		automatic_door_time_ = ros::Time(nowtime.sec + 5, nowtime.nsec);
	}

	void callbackAutomaticDoor(const std_msgs::UInt8::ConstPtr &msg)
	{
		std::cout << "automatic door : " << (int)msg->data << std::endl;
		automaticDoorSet(msg->data);
	}

	void callbackConfigMicroBusCan(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		setting_ = *msg;
		std::string safety_error_message = "";
		publisStatus(safety_error_message);
	}

	void callbackTwistCmd(const autoware_msgs::VehicleCmd::ConstPtr &msg)
	{
		std::cout << "sub twist" << std::endl;

		ros::Duration rostime = msg->header.stamp - twist_.header.stamp;
		double time_sa = rostime.sec + rostime.nsec * 1E-9;
		double ws_ave = (wheelrad_to_steering_can_value_left + wheelrad_to_steering_can_value_right) / 2.0;
		double deg = fabs(msg->ctrl_cmd.steering_angle - twist_.ctrl_cmd.steering_angle) * ws_ave * 720.0 / 15000.0;
		double zisoku = msg->ctrl_cmd.linear_velocity * 3.6;

		brake_mode_changer();

		if(dengerStopFlag == false)
		{
			bool flag = false;

			LIMIT_ANGLE_FROM_VELOCITY_STRUCT limitAngleData = lafvc.getLimit(zisoku);
			std::cout <<"angle range : " << limitAngleData.limit_angle_bottom << "," << deg << "," << limitAngleData.limit_angle_top << std::endl;

			//double targetAngleTimeVal = fabs(deg - front_deg_)/time_sa;
			std::cout << "time_sa," << time_sa << ",targetAngleTimeVal," << deg << std::endl;
			double deg_th;
			if(zisoku <= 15) deg_th = 80;
			else deg_th = 40;
			if(deg > deg_th)// && strinf.mode == MODE_PROGRAM)
			{
				if(msg->ctrl_cmd.steering_angle != 0)
				{
					flag = true;
					std::cout << "Denger! target angle over" << std::endl;
				}
			}

			if(flag == true && can_receive_502_.clutch == true)
			{
				drive_gasu_breake_ = true;
				steer_gasu_breake_ = true;
				std::stringstream safety_error_message;
				safety_error_message << "target angle over , " << deg;
				//std::cout << safety_error_message.str() << std::endl;
				publisStatus(safety_error_message.str());
			}
		}

		/*LIMIT_ANGLE_FROM_VELOCITY_STRUCT limitAngleData = lafvc.getLimit(zisoku);
		std::cout <<"angle range : " << limitAngleData.limit_angle_bottom << "," << msg->angle_deg << "," << limitAngleData.limit_angle_top << std::endl;
		if(dengerStopFlag == false)
		{
			bool flag = false;
			if((limitAngleData.limit_angle_bottom > msg->angle_deg || limitAngleData.limit_angle_top < msg->angle_deg))// &&
					//strinf.mode == MODE_PROGRAM)
			{
				flag = true;
				std::cout << "Denger! current angle limit over" << std::endl;
			}
			//long long time_sa = vstate.tstamp - stamp_backup;
			ros::Duration rostime = can_receive_502_.header.stamp - msg->header.stamp;
			double time_sa = rostime.sec + rostime.nsec * 1E-9;
			double actualAngleTimeVal = fabs(msg->angle_deg - can_receive_502_.angle_deg)/time_sa;
			std::cout << "time_sa," << time_sa << ",actualAngleTimeVal," << actualAngleTimeVal << std::endl;
			if(actualAngleTimeVal > 1.0)// && strinf.mode == MODE_PROGRAM)
			{
				flag = true;
				std::cout << "Denger! actual angle over" << std::endl;
			}
			if(flag == true)
			{
				std::stringstream safety_error_message;
				safety_error_message << "actual angle over : " << actualAngleTimeVal;
				publisStatus(safety_error_message.str());
				drive_gasu_breake_ = true;
				steer_gasu_breake_ = true;
			}
		}*/

		twist_ = *msg;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		pedal_ = msg->microbus_pedal;

		if(msg->pause != 0)
		{
			if(msg->pause > 0)
			{
				ros::Time nowtime = ros::Time::now();
				wpg_.pause_time_ = ros::Time(nowtime.sec + msg->pause, nowtime.nsec);
				//wpg_.pause_time_ = (double)clock()/CLOCKS_PER_SEC + msg->pause;
				wpg_.pause_speed_ = can_receive_501_.velocity;
			}
			else if(msg->pause < 0)
			{
				ros::Time nowtime = ros::Time::now();
				wpg_.pause_time_ = ros::Time(nowtime.sec + msg->pause, nowtime.nsec);
				//wpg_.pause_time_ = (double)clock()/CLOCKS_PER_SEC + 1000;
				wpg_.pause_speed_ = can_receive_501_.velocity;
			}
		}

		if(msg->automatic_door == 2 && msg->automatic_door != waypoint_param_.automatic_door)
		{std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			//wpg_.automatic_door_ = 2;
			//wpg_.automatic_door_taiki_ = true;
			automaticDoorSet(2);
		}
		else if(msg->automatic_door == 1 && msg->automatic_door != waypoint_param_.automatic_door)
		{std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			//wpg_.automatic_door_ = 2;
			//wpg_.automatic_door_taiki_ = true;
			automaticDoorSet(1);
		}

		if(msg->blinker == 1)
		{
			blinkerLeft();
		}
		else if(msg->blinker == 2)
		{
			blinkerRight();
		}
		else if(msg->blinker == 0)
		{
			blinkerStop();
		}

		if(msg->liesse.shift >= 0)
		{
			shift_position_ = msg->liesse.shift;
		}

		waypoint_param_ = *msg;
	}

	void callbackPositionChecker(const autoware_msgs::PositionChecker::ConstPtr &msg)
	{
		if(setting_.use_position_checker == true)
		{
			if(msg->stop_flag != 0 && can_receive_502_.clutch == true)
			{
				flag_drive_mode_ = false;
				flag_steer_mode_ = false;
				std::cout << "Denger! Autoware stop flag : " << msg->stop_flag << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "positon error : " << msg->stop_flag;
				publisStatus(safety_error_message.str());
				can_send();
			}
		}
	}

	void publisStatus(std::string safety_error_message)
	{
		autoware_can_msgs::MicroBusCanSenderStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.use_position_checker = setting_.use_position_checker;
		msg.use_input_steer = input_steer_mode_;
		msg.use_input_drive = input_drive_mode_;
		if(safety_error_message != "") msg.safety_error_message = safety_error_message;
		else msg.safety_error_message = "";
		std::cout << msg.safety_error_message << std::endl;
		pub_microbus_can_sender_status_.publish(msg);
	}

	void callbackInputSteerFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		//input_steer_ = msg->data;
		input_steer_mode_ = msg->data;
		std::cout << "aaa" << std::endl;
		std::string safety_error_message = "";
		publisStatus(safety_error_message);
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
		std::string safety_error_message = "";
		publisStatus(safety_error_message);
	}

	void callbackInputDriveValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_drive_ = msg->data;
		std::cout << input_drive_ << std::endl;
	}

	void callbackEmergencyStop(const std_msgs::UInt8::ConstPtr &msg)
	{
		emergency_stop_ = msg->data;
		std::cout << "emergency stop : " << (int)emergency_stop_ << std::endl;
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

	void callbackDriveGasuBreake(const std_msgs::Bool::ConstPtr &msg)
	{
		drive_gasu_breake_ = msg->data;
	}

	void callbackSteerGasuBreake(const std_msgs::Bool::ConstPtr &msg)
	{
		steer_gasu_breake_ = msg->data;
	}

	void blinkerRight()
	{
		blinker_right_ = true;
		std::cout << "blinker right" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_right_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerRight(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerRight();
	}

	void blinkerLeft()
	{
		blinker_left_ = true;
		std::cout << "blinker left" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_left_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerLeft(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerLeft();
	}

	void blinkerStop()
	{
		blinker_stop_ = true;
		std::cout << "blinker stop" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_stop_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerStop(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerStop();
	}

	const double braking_speed_th = 5.0;//km/h
	const double braking_separate_speed_th = 0.0;
	bool brake_flag = false;
	void brake_mode_changer()
	{
		double zisoku_twist = twist_.ctrl_cmd.linear_velocity * 3.6;
		double zisoku_can = can_receive_502_.velocity_actual / 100.0;
		std::cout << "brake : " << zisoku_twist << "," << zisoku_can << "," << (int)brake_flag << std::endl;
		if(zisoku_can <= 2.0) {brake_flag = false; return;}
		if(econtrol == EControl::STOP)
		{
			if(brake_flag == false)
			{
				if(zisoku_can - zisoku_twist >= braking_speed_th) brake_flag = true;
			}
			else
			{
				if(zisoku_twist - zisoku_can >= braking_separate_speed_th) brake_flag = false;
			}
		}
	}

	void bufset_mode(unsigned char *buf)
	{
		unsigned char mode = 0;
		if(flag_drive_mode_ == true)
		{
			if(brake_flag == true) mode |= 0x0A;
			else mode |= drive_control_mode_;
		}
		if(flag_steer_mode_ == true) mode |= 0xA0;
		buf[0] = mode;  buf[1] = 0;
	}

	double handle_control_max_speed = 50; //
	double handle_control_min_speed = 10; //
	double handle_control_ratio = 1.0/32.0;
	void bufset_steer(unsigned char *buf)
	{
		short steer_val;
		if(input_steer_mode_ == false)
		{
			/*double twist_ang = twist_.twist.angular.z*180.0 / M_PI;
			twist_ang *= HANDLE_MAX / WHEEL_MAX;
			twist_ang *= 20 * 1.0;
			steer_val = twist_ang;*/

			/*double twist_deg = twist_.twist.angular.z*180.0 / M_PI;
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
			}*/

			double wheel_ang = twist_.ctrl_cmd.steering_angle;
			double zisoku = twist_.ctrl_cmd.linear_velocity * 3.6;
			double ratio = ((handle_control_ratio - 1.0) * (zisoku - handle_control_min_speed))
			        / (handle_control_max_speed - handle_control_min_speed) + 1;
			if(wheel_ang > 0)
			{
				steer_val = wheel_ang * wheelrad_to_steering_can_value_left;// * 2;
				//if(zisoku > handle_control_min_speed) steer_val *= ratio;
			}
			else
			{
				steer_val = wheel_ang * wheelrad_to_steering_can_value_right;// * 2;
				//if(zisoku > handle_control_min_speed) steer_val *= ratio;
			}
		}
		else steer_val = input_steer_;
		if(can_receive_501_.steer_auto != autoware_can_msgs::MicroBusCan501::STEER_AUTO) steer_val = 0;

		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];
	}

	double econtrol_stop_value=0;
	void bufset_drive(unsigned char *buf)
	{
		/*double twist_drv = twist.twist.linear.x;
		short drive_val = (twist_drv / PEDAL_ACCEL_MAX) * VELOCITY_MAX;
		unsigned char *drive_point = (unsigned char*)&drive_val;
		buf[4] = drive_point[1];  buf[5] = drive_point[0];*/

		//std::cout << "mode : " << (int)drive_control_mode_ << "   vel : " << (int)MODE_VRLOCITY << std::endl;

		if(brake_flag == true)
		{
			econtrol_stop_value -= 3;
			std::cout << "obstracle : " << obstracle_waypoint_ << std::endl;
			if(obstracle_waypoint_ >= 20)
			{
				if(econtrol_stop_value < -100) econtrol_stop_value = -100;
			}
			else if(obstracle_waypoint_ <= 20 && obstracle_waypoint_ >=15)
			{
				if(econtrol_stop_value < -200) econtrol_stop_value = -200;
			}
			else if(obstracle_waypoint_ <= 15 && obstracle_waypoint_ >=10)
			{
				if(econtrol_stop_value < -350) econtrol_stop_value = -350;
			}
			else if(econtrol_stop_value < -500) econtrol_stop_value = -500;
			short pedal_val = (short)econtrol_stop_value;
			unsigned char *pedal_point = (unsigned char*)&pedal_val;
			buf[4] = pedal_point[1];  buf[5] = pedal_point[0];
			return;
		}

		econtrol_stop_value = 0;
		if(drive_control_mode_ == MODE_VERLOCITY)
		{
			short drive_val;
			if(input_drive_mode_ == false)
			{
				ros::Time nowtime = ros::Time::now();
				if(nowtime < wpg_.pause_time_)
				{
					wpg_.pause_speed_ -= 50;
					if(wpg_.pause_speed_ < -300) wpg_.pause_speed_ = -300;
					drive_val = wpg_.pause_speed_;
				}
				else
				{
					double linearx = twist_.ctrl_cmd.linear_velocity;
					//if(linearx > 0.5 && linearx < 1.0) linearx = 1.0;
					double twist_drv = linearx *3.6 * 100; //std::cout << twist_drv << std::endl;
					drive_val = twist_drv;
				}
			}
			else drive_val = input_drive_;
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_val = 0;
			if(can_receive_503_.clutch == false)
				drive_val = can_receive_502_.velocity_actual;

			unsigned char *drive_point = (unsigned char*)&drive_val;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
		else
		{
			short pedal_val;
			if(input_drive_mode_ == false)
			{
				//short drive_val = input_drive_;
				unsigned char pedal_val = pedal_;
			}
			else pedal_val = input_drive_;
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				pedal_val = 0;
			unsigned char *pedal_point = (unsigned char*)&pedal_val;
			buf[4] = pedal_point[1];  buf[5] = pedal_point[0];
		}
	}

	void bufset_car_control(unsigned char *buf)
	{
		buf[6] = buf[7] = 0;

		if(emergency_stop_ == 0x2) {buf[6] |= 0x80;  emergency_stop_ = 0;}
		else if(emergency_stop_ == 0x1) {buf[6] |= 0x40;  emergency_stop_ = 0;}
		if(drive_gasu_breake_ == true) {buf[6] |= 0x20;}// drive_gasu_breake_ = false;}
		if(steer_gasu_breake_ == true) {buf[6] |= 0x10;}// steer_gasu_breake_ = false;}
		if(automatic_door_ != 0x0)
		{
			std::cout << "aaaaaaaaaaaaaaaaaaaaa : " << automatic_door_time_ << std::endl;
			if(automatic_door_ == 0x2) {buf[6] |= 0x08;}
			else if(automatic_door_ == 0x1) {buf[6] |= 0x04;}
			ros::Time time = ros::Time::now();
			if(time > automatic_door_time_)  automatic_door_ = 0x0;
		}
		if(blinker_right_ == true)
		{std::cout << "aaaaaaaaaaaaaaaaaaaaa" << std::endl;
			buf[6] |= 0x02; //blinker_right_ = false;
			ros::Time time = ros::Time::now();
			if(time > blinker_right_time_)  blinker_right_ = false;
		}
		else if(blinker_left_ == true)
		{
			buf[6] |= 0x01;
			ros::Time time = ros::Time::now();
			if(time > blinker_left_time_)  blinker_left_ = false;
		}
		else if(blinker_stop_ == true)
		{
			buf[6] |= 0x03;
			ros::Time time = ros::Time::now();
			if(time > blinker_stop_time_)  blinker_stop_ = false;
		}
		if(hazard_ == true) buf[7] |= 0x80;
		if(horn_ == true) buf[7] |= 0x40;
		if(light_high_ == true) buf[7] |= 0x20;
		if(engine_start_ == true) buf[7] |= 0x10;
		//if(ignition_ == true) buf[6] |= 0x10;
		//if(wiper_ == true) buf[6] |= 0x08;
		//if(light_low_ == true) buf[6] |= 0x02;
		//if(light_small_ == true) buf[6] |= 0x01;

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
	    , input_drive_mode_(true)
	    , input_steer_mode_(true)
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
	    , blinker_stop_(false)
	    , automatic_door_(0)
	    , drive_gasu_breake_(false)
	    , steer_gasu_breake_(false)
	{
		can_receive_501_.emergency = true;
		can_receive_501_.blinker_right = can_receive_501_.blinker_left = false;
		canStatus res = kc.init(kvaser_channel, canBITRATE_500K);
		if(res != canStatus::canOK) {std::cout << "open error" << std::endl;}

		setting_.use_position_checker = true;

		lafvc.add(limit10); lafvc.add(limit15); lafvc.add(limit20); lafvc.add(limit30); lafvc.add(limit40);

		pub_microbus_can_sender_status_ = nh_.advertise<autoware_can_msgs::MicroBusCanSenderStatus>("/microbus/can_sender_status", 10, true);

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);
		sub_twist_cmd_ = nh_.subscribe("/vehicle_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		sub_microbus_can_501_ = nh_.subscribe("/microbus/can_receive501", 10, &kvaser_can_sender::callbackMicrobusCan501, this);
		sub_microbus_can_502_ = nh_.subscribe("/microbus/can_receive502", 10, &kvaser_can_sender::callbackMicrobusCan502, this);
		sub_microbus_can_503_ = nh_.subscribe("/microbus/can_receive503", 10, &kvaser_can_sender::callbackMicrobusCan503, this);
		sub_emergency_reset_ = nh_.subscribe("/microbus/emergency_reset", 10, &kvaser_can_sender::callbackEmergencyReset, this);
		sub_input_steer_flag_ = nh_.subscribe("/microbus/input_steer_flag", 10, &kvaser_can_sender::callbackInputSteerFlag, this);
		sub_input_steer_value_ = nh_.subscribe("/microbus/input_steer_value", 10, &kvaser_can_sender::callbackInputSteerValue, this);
		sub_input_drive_flag_ = nh_.subscribe("/microbus/input_drive_flag", 10, &kvaser_can_sender::callbackInputDriveFlag, this);
		sub_input_drive_value_ = nh_.subscribe("/microbus/input_drive_value", 10, &kvaser_can_sender::callbackInputDriveValue, this);
		sub_stroke_mode_ = nh_.subscribe("/microbus/set_stroke_mode", 10, &kvaser_can_sender::callbackStrokeMode, this);
		sub_velocity_mode_ = nh_.subscribe("/microbus/set_velocity_mode", 10, &kvaser_can_sender::callbackVelocityMode, this);
		sub_drive_control_ = nh_.subscribe("/microbus/drive_control_", 10, &kvaser_can_sender::callbackDriveControl, this);
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
		sub_blinker_stop_ = nh_.subscribe("/microbus/blinker_stop", 10, &kvaser_can_sender::callbackBlinkerStop, this);
		sub_automatic_door_ = nh_.subscribe("/microbus/automatic_door", 10, &kvaser_can_sender::callbackAutomaticDoor, this);
		sub_drive_gasu_breake_ = nh_.subscribe("/microbus/drive_gasu_breake", 10, &kvaser_can_sender::callbackDriveGasuBreake, this);
		sub_steer_gasu_breake_ = nh_.subscribe("/microbus/steer_gasu_breake", 10, &kvaser_can_sender::callbackSteerGasuBreake, this);
		sub_econtrol_ = nh_.subscribe("/econtrol", 10, &kvaser_can_sender::callbackEControl, this);
		sub_obtracle_waypoint_ = nh_.subscribe("/obstacle_waypoint", 10, &kvaser_can_sender::callbackObstracleWaypoint, this);

		std::string safety_error_message = "";
		publisStatus(safety_error_message);

		proc_time.sec = 0;
		proc_time.nsec = 0;
		waypoint_param_.blinker = 0;
		automatic_door_time_ = blinker_right_time_ = blinker_left_time_ =
		        blinker_stop_time_ = ros::Time::now();
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

			/*if(wpg_.automatic_door_ == 2 && wpg_.automatic_door_taiki_ == true)
			{
				//if(can_receive_502_.velocity_actual < 150)
				{
					automaticDoorSet(2);
					wpg_.automatic_door_ = 0;
					wpg_.automatic_door_taiki_ = false;
				}
			}*/
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
