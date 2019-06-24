#include <canlib.h>

const unsigned long READ_WAIT_INFINITE = -1;

class KVASER_CAN
{
private:
	canHandle can_handle_;
	bool open_flag_;
	unsigned int msgCounter_;
public:
	KVASER_CAN()
	    : can_handle_(NULL)
	    , msgCounter_(0)
	    , open_flag_(false)
	{
	}

	canStatus init(int kvaser_channel)
	{
		canInitializeLibrary();
		can_handle_ = canOpenChannel(kvaser_channel, canOPEN_CAN_FD);
		if(can_handle_ != canStatus::canOK) return (canStatus)can_handle_;

		canStatus res = canSetBusParams(can_handle_, canBITRATE_500K, 0, 0, 0, 0, 0);
		if(canStatus::canOK != res) return res;

		res = canBusOn(can_handle_);
		if(canStatus::canOK != res) return res;

		open_flag_ = true;
		return canStatus::canOK;
	}

	~KVASER_CAN()
	{
		if(can_handle_ == canStatus::canOK)
		canBusOff(can_handle_);
	}

	bool isOpen() {return open_flag_;}

	canStatus read_wait()
	{
		long id;
		unsigned char msg[8];
		unsigned int dlc;
		unsigned int flag;
		unsigned long time;

		canStatus res = canReadWait(can_handle_, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);
		if(canStatus::canOK != res) return res;

		if (flag & canMSG_ERROR_FRAME)
		{
			std::cerr << "error frame" << std::endl;
			return canStatus::canOK;
		}

		msgCounter_++;
		printf("(%u) id:%lx dlc:%u data: ", msgCounter_, id, dlc);
		if (dlc > 8) {
		  dlc = 8;
		}
		for (unsigned int j = 0; j < dlc; j++) {
		  printf("%2.2x ", msg[j]);
		}
		printf(" flags:0x%x time:%lu\n", flag, time);
		fflush(stdout);

		/*if(id == 0x501)
		{
			short *short_buf = (short*)msg;
			unsigned short *ushort_buf = (unsigned short*)msg;
			std::cout << "デジタル出力状態 : " << ushort_buf[0] << std::endl;
			std::cout << "操舵角度実際値 : " << short_buf[1] << std::endl;
			std::cout << "車両速度実際値 : " << short_buf[2] << std::endl;
			std::cout << "運転状態 : " << ushort_buf[3] << std::endl;
		}*/
		return canStatus::canOK;
	}

	canStatus write(long id, char *data, unsigned int size)
	{
		canStatus res = canWrite(can_handle_, id, data, size, 0);
		if(canStatus::canOK != res) return res;

		return canStatus::canOK;
	}
};
