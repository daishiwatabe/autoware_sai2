#include <ros/ros.h>
#include <HevControl.h>
#include <CANUSB.h>

struct BrakeInf {
	bool pressed;           // ペダルスイッチ状態(ON=true, OFF=false)
	int actualPedalStr;		// ペダルストローク現在値
	int targetPedalStr;		// ペダルストローク目標値
	int inputPedalStr;		// ペダルストローク入力値
	float prl;              // PRLセンサ値
	float pfl;              // PFLセンサ値
	float prr;              // PRRセンサ値
	float pfr;              // PFRセンサ値
	float sks1;             // SKS1ペダルセンサ値
	float sks2;             // SKS2ペダルセンサ値
	float pmc1;             // PMC1ペダルセンサ値
	float pmc2;             // PMC2ペダルセンサ値
//    int targetStr;		// 目標ストローク
	unsigned char brakeLamp;
	unsigned char blinkerLeft;
	unsigned char blinkerRight;
	unsigned char brakeMode;

};

struct OtherInf {
	float sideAcc;          // 0x22 Yawレート
	float acc;              // 0x23 前後加速度
	float angleFromP;       // 0x25 ステアリング角度
	float brkPedalStrFromP;// 0x30 ブレーキペダル状態
	float velocFrFromP;		// 0xB1 右前輪速度[km/h*100]
	float velocFlFromP;		// 0xB1 左前輪速度[km/h*100]
	float velocRrFromP;		// 0xB3 右後輪速度[km/h*100]
	float velocRlFromP;		// 0xB3 左後輪速度[km/h*100]
	float velocFromP2;      // 0xB4 速度
	int drv_mode;           // 0x120 ドライブモード
	int drvPedalStrFromP;  // 0x244 アクセルペダル状態
	int rpm;                // 0x3C8 エンジンの回転数
	float velocFromP;       // 0x3CA 速度
	int ev_mode;            // 0x529 EVモード
	int temp;               // 0x52C 温度[℃ ]
	int shiftFromPrius;     // 0x540 シフト状態
	zmp::hev::LIGHT_STATE light;		// 0x57F ライト状態
	int level;              // 0x5A4 燃料の残量
	zmp::hev::DOOR_STATE door;		// 0x5B6 ドアの状態
	bool cluise;            // 0x5C8 クルーズコントロールON/OFF
	char dtcData1;          // 0x7E8,0x7EA,0x7EB
	char dtcData2;          // 0x7E8,0x7EA,0x7EB
	char dtcData3;          // 0x7E8,0x7EA,0x7EB
	char dtcData4;          // 0x7E8,0x7EA,0x7EB
	char dtcData5;          // 0x7E8,0x7EA,0x7EB
	char dtcData6;          // 0x7E8,0x7EA,0x7EB
	char dtcData7;          // 0x7E8,0x7EA,0x7EB
	char dtcData8;          // 0x7E8,0x7EA,0x7EB
};

struct StrInf {
	int mode;           // ステアリングモード(manual=0x00, program=0x10)
	int cont_mode;      // ステアリング制御モード(torque=0x00, angle=0x10)
	int overrideMode;   // オーバーライドモード(ON=0x00, OFF=0x10)
	int servo;          // 制御のON/OFF(ON=true, OFF=false)
	int targetTorque;   // 目標トルク
	int torque;         // 操舵トルク
	float trq1;         // TRQ1トルクセンサ
	float trq2;         // TRQ2トルクセンサ
	float angle;        // 操舵角度[deg * 10]
	float targetAngle;  // 目標操舵角[deg*10]
};

class canusbz_can_receiver : public zmp::hev::ChangeStateObserver
{
private:
	ros::NodeHandle nh_, private_nh_;
	zmp::hev::CANUSBZ* canCom_;// = new zmp::hev::CANUSBZ();
	zmp::hev::HevControl* hevCnt_;// = new zmp::hev::HevControl();

	StrInf strInf_;
	OtherInf otherInf_;
public:
	canusbz_can_receiver(ros::NodeHandle nh, ros::NodeHandle p_nh)
	{
		nh_ = nh;  private_nh_ = p_nh;

		canCom_ = new zmp::hev::CANUSBZ();
		hevCnt_ = new zmp::hev::HevControl();

		std::string device_name = "/dev/ttyACM0";
		int res = hevCnt_->InitHevControl(canCom_, (char*)device_name.c_str());
		std::cout << "open " << device_name << " : " << res << std::endl;

		bool boolres = canCom_->SetCANUSBZParam(CAN_CHANNEL_0, CAN_SPEED_500, CANID_KIND_11);
		std::cout << "CAN_CHANNEL_0 : " << bool2string(boolres) << std::endl;
		boolres = canCom_->SetCANUSBZParam(CAN_CHANNEL_1, CAN_SPEED_1000, CANID_KIND_11);
		std::cout << "CAN_CHANNEL_1 : " << bool2string(boolres) << std::endl;

		res = hevCnt_->SetStatusCallback(this);
		std::cout << "set callback : " << hevres2string(res) << std::endl;

		hevCnt_->SetStrControlMode(0x10);
		hevCnt_->SetStrOverrideMode(zmp::hev::OVERRIDE_MODE_ON);
		hevCnt_->SetStrAngle(0.0f);
		hevCnt_->SetStrServo(0x10);
		hevCnt_->SetStrAngle(0.0f);

		canCom_->Start();
	}

	~canusbz_can_receiver()
	{
		canCom_->Stop();
		delete hevCnt_;
		delete canCom_;
	}

	std::string bool2string(bool b) {return (b == true) ? "true" : "false";}
	std::string hevres2string(int res) {return (res == 0) ? "true" : "false";}

	void UpdateSteerState(zmp::hev::REP_STEER_INFO_INDEX index)
	{std::cout << "zzz" << std::endl;
		switch(index){
		case zmp::hev::REP_STR_MODE:
			hevCnt_->GetStrMode((int&)strInf_.mode);
			hevCnt_->GetStrControlMode((int&)strInf_.cont_mode);
			hevCnt_->GetStrOverrideMode((int&)strInf_.overrideMode);
			hevCnt_->GetStrServo((int&)strInf_.servo);
			break;
		case zmp::hev::REP_TORQUE:
			hevCnt_->GetStrTorque((int&)strInf_.torque);
			hevCnt_->GetStrTargetTorque((int&)strInf_.targetTorque);
	//        _hevCnt->SetStrTorque(_strInf.targetTorque + _asistTrq);
			break;
		case zmp::hev::REP_ANGLE: hevCnt_->GetStrAngle((float&)strInf_.angle, (float&)strInf_.targetAngle); break;
		case zmp::hev::REP_ANGLE_FROMOBD: hevCnt_->GetStrAngleFromOBD((float&)otherInf_.angleFromP); break;
		default: printf("\n"); break;
		}

		return;
	}
	void UpdateDriveState(zmp::hev::REP_DRIVE_INFO_INDEX index)
	{
		std::cout << "aaa" << std::endl;
	}
	void UpdateBattState(zmp::hev::REP_BATT_INFO_INDEX index)
	{
		std::cout << "bbb" << std::endl;
	}
	void UpdateOtherState(zmp::hev::REP_OTHER_INFO_INDEX index)
	{
		std::cout << "ccc" << std::endl;
	}
	void UpdateDemoSensorState(zmp::hev::REP_DEMO_SENSOR_INFO_INDEX index)
	{
		std::cout << "ddd" << std::endl;
	}
	void ReceiveConfig(int num, int index, int value[])
	{
		std::cout << "eee" << std::endl;
	}
	void ReceiveErrorStatus(int level, int errCode)
	{
		std::cout << "fff" << std::endl;
	}
	void ReceiveEcho(int ctlKind, int ctlNo)
	{
		std::cout << "ggg" << std::endl;
	}
	void ReceiveImuMsg(zmp::hev::REP_IMU_INFO_INDEX index)
	{
		std::cout << "hhh" << std::endl;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "canusbz_prius_can_receiver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	canusbz_can_receiver ccr(nh, private_nh);

	while(ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
