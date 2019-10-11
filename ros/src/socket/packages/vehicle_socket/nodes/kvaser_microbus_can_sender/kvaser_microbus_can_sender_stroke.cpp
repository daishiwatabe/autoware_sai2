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

class PID_params
{
private:
	double accel_e_prev_, brake_e_prev_;
	double accel_diff_sum_, brake_diff_sum_;
	short stroke_prev_, stop_stroke_prev_;
public:
	PID_params() {}
	int init(short stop_stroke)
	{
		accel_e_prev_ = brake_e_prev_ = accel_diff_sum_ = brake_diff_sum_ = 0;
		stroke_prev_ = stop_stroke_prev_ = stop_stroke;
	}

	void clear_diff()
	{
		accel_diff_sum_ = brake_diff_sum_ = 0;
	}

	double get_accel_e_prev() {return accel_e_prev_;}
	double get_brake_e_prev() {return brake_e_prev_;}
	double get_acclel_diff_sum() {return accel_diff_sum_;}
	double get_brake_diff_sum() {return brake_diff_sum_;}
	short get_stop_stroke_prev() {return stop_stroke_prev_;}
	short get_stroke_prev() {return stroke_prev_;}

	void set_accel_e_prev(double val) {accel_e_prev_ = val;}
	void set_brake_e_prev(double val) {brake_e_prev_ = val;}
	void plus_accel_diff_sum(double val) {accel_diff_sum_ += val;}
	void plus_brake_diff_sum(double val) {brake_diff_sum_ += val;}
	void set_stop_stroke_prev(short val) {stop_stroke_prev_ = val;}
	void set_stroke_prev(short val) {stroke_prev_ = val;}
};

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
	char automatic_door_;
	bool automatic_door_taiki_;

	waypoint_param_geter()
	{
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
	//velcity params
	const short VELOCITY_ZERO_VALUE_ = 140;

	//stroke params
	const short PEDAL_VOLTAGE_CENTER_ = 1024;//計測値は1025;
	const short PEDAL_DISPLACEMENT_CENTER_ = 1024;//計測値は1029;
	const short PEDAL_VOLTAGE_MAX_ = 161;
	const short PEDAL_DISPLACEMENT_MAX_ = 1161;
	const short PEDAL_VOLTAGE_MIN_ = 1533;
	const short PEDAL_DISPLACEMENT_MIN_ = 849;
	//const short PEDAL_STROKE_CENTER_ = 0;
	//const short PEDAL_STOKE_MAX_ = PEDAL_VOLTAGE_CENTER_ - PEDAL_VOLTAGE_MAX_;
	//const short PEDAL_STROKE_MIN_ = -450;//PEDAL_VOLTAGE_CENTER_ - PEDAL_VOLTAGE_MIN_;
	//const short ACCEL_PEDAL_STROKE_OFFSET_ = 10;
	//const short BRAKE_PEDAL_STROKE_OFFSET_ = -10;
	//const short BRAKE_PEDAL_STOPPING_MED_ = 400;
	//const short BRAKE_PEDAL_STOPPING_MAX_ = 500;

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
	const static unsigned char SHIFT_4 = 4;
	const static unsigned char SHIFT_L = 5;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;

	//error params
	const int CONFIG_OK = 0;
	const int ERROR_STROKE_MAX_MIN_INCONSISTENCY = 1;
	const int ERROR_STROKE_CENTER_INCONSISTENCY = 2;
	const int ERROR_STROKE_STOPPING_MED_INCONSISTENCY = 3;
	const int ERROR_STROKE_ACCEL_STROKE_INCONSISTENCY = 4;
	const int ERROR_STROKE_BRAKE_STROKE_INCONSISTENCY = 5;
	int config_result;

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
	ros::Subscriber sub_emergency_stop_;
	ros::Subscriber sub_light_high_, sub_light_low_, sub_light_small_;
	ros::Subscriber sub_blinker_right_, sub_blinker_left_, sub_blinker_stop_;
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
	bool light_high_, light_low_, light_small_;
	bool blinker_right_, blinker_left_, blinker_stop_;
	EControl econtrol;
	int obstracle_waypoint_;
	autoware_msgs::WaypointParam waypoint_param_;
	PID_params pid_params;

	ros::Time automatic_door_time_;
	ros::Time blinker_right_time_, blinker_left_time_, blinker_stop_time_;

	waypoint_param_geter wpg_;

	void callbackEmergencyStop(const std_msgs::UInt8::ConstPtr &msg)
	{
		emergency_stop_ = msg->data;
		std::cout << "emergency stop : " << (int)emergency_stop_ << std::endl;
	}

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
	}

	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
	{
		std::cout << "sub can_503" << std::endl;
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

		/*setting_.pedal_stroke_max - setting_.pedal_stroke_min
		if(setting_.pedal_stroke_max - setting_.pedal_stroke_min > 1300 ||
				)
			config_result = ERROR_STROKE_MAX_MIN_INCONSISTENCY;
		else if(setting_.pedal_stroke_max - setting_.pedal_stroke_center )*/
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

		//クラッチが入っている(autoモードである)場合だけエラー判定を出す
		if(flag == true && can_receive_502_.clutch == true)
		{
			drive_gasu_breake_ = true;
			steer_gasu_breake_ = true;
			std::stringstream safety_error_message;
			safety_error_message << "target angle over , " << deg;
			//std::cout << safety_error_message.str() << std::endl;
			publisStatus(safety_error_message.str());
		}

		twist_ = *msg;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		pedal_ = msg->microbus_pedal;

		if(msg->automatic_door == 2 && msg->automatic_door != waypoint_param_.automatic_door)
		{
			std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			automaticDoorSet(2);
		}
		else if(msg->automatic_door == 1 && msg->automatic_door != waypoint_param_.automatic_door)
		{
			std::cout << "automatic_door : " << msg->automatic_door << std::endl;
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

	void bufset_mode(unsigned char *buf)
	{
		unsigned char mode = 0;
		if(flag_drive_mode_ == true) mode |= drive_control_mode_;
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
			double wheel_ang = twist_.ctrl_cmd.steering_angle;
			double zisoku = twist_.ctrl_cmd.linear_velocity * 3.6;
			double ratio = ((handle_control_ratio - 1.0) * (zisoku - handle_control_min_speed))
			        / (handle_control_max_speed - handle_control_min_speed) + 1;
			if(wheel_ang > 0)
			{
				steer_val = wheel_ang * wheelrad_to_steering_can_value_left;
			}
			else
			{
				steer_val = wheel_ang * wheelrad_to_steering_can_value_right;
			}
		}
		else steer_val = input_steer_;
		if(can_receive_501_.steer_auto != autoware_can_msgs::MicroBusCan501::STEER_AUTO) steer_val = 0;

		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];
	}

	short _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
	{
		//ブレーキからアクセルに変わった場合、Iの積算値をリセット
		short stroke = PEDAL_VOLTAGE_CENTER_ - can_receive_503_.pedal_voltage;
		if(stroke < setting_.brake_stroke_offset)
		{
			pid_params.set_accel_e_prev(0);
			return 0;
		}

		//P
		double e = twist_.ctrl_cmd.linear_velocity - can_receive_502_.velocity_average;

		//I
		double e_i;
		pid_params.plus_accel_diff_sum(e);
		if (pid_params.get_acclel_diff_sum() > setting_.accel_max_i)
			e_i = setting_.accel_max_i;
		else
			e_i = pid_params.get_acclel_diff_sum();

		//D
		double e_d = e - pid_params.get_accel_e_prev();

		double target_accel_stroke = setting_.k_accel_p_until * e +
		       setting_.k_accel_i_until * e_i +
		       setting_.k_accel_d_until * e_d;

		short ret = (short)(target_accel_stroke + 0.5);
		if(ret > setting_.pedal_stroke_max)
			ret = setting_.pedal_stroke_max;
		else if (ret < setting_.pedal_stroke_center)
			ret = setting_.pedal_stroke_center;

		pid_params.set_accel_e_prev(e);
		return ret;
	}

	short _brake_stroke_pid_control(double current_velocity, double cmd_velocity)
	{
		//std::cout << "cur" << current_velocity << "  cmd" << cmd_velocity << std::endl;
		//アクセルからブレーキに変わった場合、Iの積算値をリセット
		short stroke = PEDAL_VOLTAGE_CENTER_ - can_receive_503_.pedal_voltage;
		std::cout << "stroke " << stroke << std::endl;
		if (stroke > setting_.accel_stroke_offset)
		{
			std::cout << "ACCEL_PEDAL_STROKE_OFFSET_" << std::endl;
			pid_params.set_brake_e_prev(0);
			return 0;
		}

		//P
		double e = -1 * (cmd_velocity - current_velocity);
		// since this is braking, multiply -1.
		if (e > 0 && e <= 1) { // added @ 2016/Aug/29
			e = 0;
			pid_params.clear_diff();
		}
		std::cout << "e " << e << std::endl;
		//I
		double e_i;
		pid_params.plus_brake_diff_sum(e);
		if (pid_params.get_brake_diff_sum() > setting_.brake_max_i)
			e_i = setting_.brake_max_i;
		else
			e_i = pid_params.get_brake_diff_sum();

		//D
		double e_d = e - pid_params.get_brake_e_prev();

		double target_brake_stroke = setting_.k_brake_p_until * e +
		        setting_.k_brake_i_until * e_i +
		        setting_.k_brake_d_until * e_d;

		short ret = -1 * (short)(target_brake_stroke + 0.5);std::cout << "ret " << setting_.k_brake_p_until << std::endl;
		if (ret < setting_.pedal_stroke_min)
			ret = setting_.pedal_stroke_min;
		else if (ret > setting_.pedal_stroke_center)
			ret = setting_.pedal_stroke_center;

		pid_params.set_brake_e_prev(e);
		return ret;
	}

	short _stopping_control(double current_velocity)
	{
		if (current_velocity < 0.25)
		{
			int gain = (int)(((double)setting_.pedal_stroke_min)*can_receive_502_.cycle_time);
			double ret = pid_params.get_stop_stroke_prev() + gain;
			if((int)ret > setting_.pedal_stroke_min) ret = setting_.pedal_stroke_min;
			return ret;
		}
		else
		{
			return setting_.brake_stroke_stopping_med;
		}
	}

	void bufset_drive(unsigned char *buf)
	{
		if(can_receive_501_.drive_mode == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY)
		{
			short drive_val;
			if(input_drive_mode_ == false)
			{
				std::cout <<"jjj : " << twist_.ctrl_cmd.linear_velocity;
				double linearx = twist_.ctrl_cmd.linear_velocity;
				double twist_drv = linearx *3.6 * 100;
				drive_val = twist_drv;
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
			double current_velocity = can_receive_502_.velocity_average / 100.0;
			double cmd_velocity;
			if(input_drive_mode_ == false)
				cmd_velocity = twist_.ctrl_cmd.linear_velocity * 3.6;
			else
				cmd_velocity = input_drive_ / 100.0;

			//AUTOモードじゃない場合、stroke値0をcanに送る
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
			{
				pid_params.clear_diff();
				short drive_val = 0;
				unsigned char *drive_point = (unsigned char*)&drive_val;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
				return;
			}

			//加速判定
			if (fabs(cmd_velocity) > current_velocity
			        && fabs(cmd_velocity) > 0.0
			        && current_velocity < setting_.velocity_limit)
			{
				std::cout << "stroke drive" << std::endl;
				//short accel_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);
				short drive_stroke = 0;
				pid_params.set_stroke_prev(drive_stroke);
				unsigned char *drive_point = (unsigned char*)&drive_stroke;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
			}
			//減速判定
			else if(fabs(cmd_velocity) < current_velocity
			         && fabs(cmd_velocity) > 0.0)
			{
				std::cout << "stroke brake" << std::endl;
				short brake_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
				pid_params.set_stroke_prev(brake_stroke);
				unsigned char *drive_point = (unsigned char*)&brake_stroke;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
			}
			//停止判定
			else if(cmd_velocity == 0.0 && current_velocity > VELOCITY_ZERO_VALUE_/100.0)
			{
				if(current_velocity < setting_.velocity_stop_th)
				{
					short brake_stroke = _stopping_control(current_velocity);
					pid_params.set_stroke_prev(brake_stroke);
					unsigned char *drive_point = (unsigned char*)&brake_stroke;
					buf[4] = drive_point[1];  buf[5] = drive_point[0];
				}
				else
				{
					short brake_stroke = _brake_stroke_pid_control(current_velocity, 0);
					pid_params.set_stroke_prev(brake_stroke);
					pid_params.set_stop_stroke_prev(brake_stroke);
					unsigned char *drive_point = (unsigned char*)&brake_stroke;
					buf[4] = drive_point[1];  buf[5] = drive_point[0];
				}
			}
			else
			{
				short drive_stroke = pid_params.get_stroke_prev();
				unsigned char *drive_point = (unsigned char*)&drive_stroke;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
			}
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
			if(automatic_door_ == 0x2) {buf[6] |= 0x08;}
			else if(automatic_door_ == 0x1) {buf[6] |= 0x04;}
			ros::Time time = ros::Time::now();
			if(time > automatic_door_time_)  automatic_door_ = 0x0;
		}
		if(blinker_right_ == true)
		{
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
		if(light_high_ == true) buf[7] |= 0x20;

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
			case SHIFT_4:
				buf[7] |= 0x04;
				break;
			case SHIFT_L:
				buf[7] |= 0x05;
				break;
			}
		}
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
	    , shift_auto_(false)
	    , shift_position_(0)
	    , emergency_stop_(false)
	    , light_high_(false)
	    , light_low_(false)
	    , light_small_(false)
	    , blinker_right_(false)
	    , blinker_left_(false)
	    , blinker_stop_(false)
	    , automatic_door_(0)
	    , drive_gasu_breake_(false)
	    , steer_gasu_breake_(false)
	    , config_result(CONFIG_OK)
	{
		setting_.use_position_checker = true;
		setting_.velocity_limit = 50;
		setting_.velocity_stop_th = 4.0;
		setting_.accel_max_i = 3000.0;
		setting_.brake_max_i = 500.0;
		setting_.k_accel_p_until = 120.0;
		setting_.k_accel_i_until = 0.1;
		setting_.k_accel_d_until = 0.1;
		setting_.k_brake_p_until = 40.0;
		setting_.k_brake_i_until = 10.0;
		setting_.k_brake_d_until = 10.0;
		setting_.pedal_stroke_center = 0;
		setting_.pedal_stroke_max = 850;
		setting_.pedal_stroke_min = -500;
		setting_.brake_stroke_stopping_med = 400;

		can_receive_501_.emergency = true;
		can_receive_501_.blinker_right = can_receive_501_.blinker_left = false;
		canStatus res = kc.init(kvaser_channel, canBITRATE_500K);
		if(res != canStatus::canOK) {std::cout << "open error" << std::endl;}

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
		sub_light_high_ = nh_.subscribe("/microbus/light_high", 10, &kvaser_can_sender::callbackLightHigh, this);
		sub_light_low_ = nh_.subscribe("/microbus/light_low", 10, &kvaser_can_sender::callbackLightLow, this);
		sub_light_small_ = nh_.subscribe("/microbus/light_small", 10, &kvaser_can_sender::callbackLightSmall, this);
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

		waypoint_param_.blinker = 0;
		automatic_door_time_ = blinker_right_time_ = blinker_left_time_ =
		        blinker_stop_time_ = ros::Time::now();

		pid_params.init(setting_.brake_stroke_stopping_med);
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
