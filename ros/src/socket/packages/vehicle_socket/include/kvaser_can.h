#include <canlib.h>

class KVASER_CAN
{
public:
	static const unsigned int READ_DATA_SIZE = 8;
	static const unsigned long READ_WAIT_INFINITE = -1;
private:
	canHandle can_handle_;
	bool open_flag_;
	unsigned int msgCounter_;
	unsigned char read_data_[READ_DATA_SIZE];
	long id_;
	unsigned int dlc_, flag_;
	unsigned long time_;
public:
	KVASER_CAN()
	    : can_handle_(NULL)
	    , msgCounter_(0)
	    , open_flag_(false)
	{
	}

	canStatus init(int kvaser_channel, int canBitrate)
	{
		canInitializeLibrary();
		can_handle_ = canOpenChannel(kvaser_channel, canOPEN_CAN_FD);
		//if(can_handle_ != canStatus::canOK) return (canStatus)can_handle_;

		canStatus res = canSetBusParams(can_handle_, canBitrate, 0, 0, 0, 0, 0);
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

	void printReader()
	{
		printf("(%u) id:%lx dlc:%u data: ", msgCounter_, id_, dlc_);
		if (dlc_ > READ_DATA_SIZE) {
		  dlc_ = READ_DATA_SIZE;
		}
		for (unsigned int j = 0; j < dlc_; j++) {
		  printf("%2.2x ", read_data_[j]);
		}
		printf(" flags:0x%x time:%lu\n", flag_, time_);
		fflush(stdout);
	}

	canStatus read_wait(unsigned long wait_time)
	{
		canStatus res = canReadWait(can_handle_, &id_, &read_data_, &dlc_, &flag_, &time_, wait_time);
		if(canStatus::canOK != res) {
			//std::cout << res << std::endl;
			return res;
		}

		msgCounter_++;
		if (flag_ & canMSG_ERROR_FRAME)
		{
			std::cerr << "error frame" << std::endl;
			return res;
		}

		//if(id_ == 0x501 || id_ == 0x502 || id_ == 0x100)
		/*{
		printf("(%u) id:%lx dlc:%u data: ", msgCounter_, id_, dlc);
		if (dlc > READ_DATA_SIZE) {
		  dlc = READ_DATA_SIZE;
		}
		for (unsigned int j = 0; j < dlc; j++) {
		  printf("%2.2x ", read_data_[j]);
		}
		printf(" flags:0x%x time:%lu\n", flag, time);
		fflush(stdout);
		}*/

		/*if(id_ == 0x501)
		{
			short *short_buf = (short*)msg;
			unsigned short *ushort_buf = (unsigned short*)read_data_;
			std::cout << "デジタル出力状態 : " << ushort_buf[0] << std::endl;
			std::cout << "車両速度実際値 : " << short_buf[1] << std::endl;
			std::cout << "操舵角度実際値 : " << short_buf[2] << std::endl;
			std::cout << "運転状態 : " << ushort_buf[3] << std::endl;
		}*/
		return res;
	}

	canStatus write(long id, char *data, unsigned int size)
	{
		canStatus res = canWrite(can_handle_, id, data, size, 0);
		if(canStatus::canOK != res) return res;

		return canStatus::canOK;
	}

	void get_read_data(unsigned char* buf)
	{
		for(unsigned int i=0; i<READ_DATA_SIZE; i++)
			buf[i] = read_data_[i];
	}

	long get_id() { return id_; }

	unsigned int get_read_counter() { return msgCounter_; }
};
