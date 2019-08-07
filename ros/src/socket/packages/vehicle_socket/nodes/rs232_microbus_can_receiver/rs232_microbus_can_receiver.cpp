#include <ros/ros.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

class socket_class
{
private:
	std::string ip_;
	int port_;
	int sock_, client_sock_;
	bool socket_connect_, bind_check_;
public:
	socket_class()
	    : ip_("")
	    , port_(0)
	    , sock_(-1)
	    , socket_connect_(false)
	{}

	~socket_class()
	{
		if(socket_connect_ == true) close(sock_);
	}

	void socket_connect(std::string ip, int port)
	{
		ip_ = ip;  port_ = port;

		sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);//ソケットの作成
		if(socket == 0)
		{
			std::cout << "error : socket function" << std::endl;
			return;
		}
		socket_connect_ = true;
		//std::cout << "sock : " << sock_ << std::endl;

		ifreq ifr;

		strcpy(ifr.ifr_name, "can0");
		int ref = ioctl(sock_, SIOCGIFINDEX, &ifr);
		if(ref == -1)
		{
			std::cout << "error : ioctl : " << errno << std::endl;
			return;
		}

		//接続先指定用構造体の準備
		struct sockaddr_can addr;
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		std::cout << "index : " << ifr.ifr_ifindex << std::endl;
		ref = bind(sock_, (struct sockaddr *)&addr, sizeof(addr));
		if(ref == 0)
		{
			std::cout << "error : bind" << std::endl;
			return;
		}
	}

	const bool isConnect()
	{
		if(socket_connect_==true && bind_check_==true) return true;
		else return false;
	}

	int reader()
	{
		unsigned char read_data[8];
		std::cout << "aaa" << std::endl;
		int ref = read(sock_, read_data, 1);
		std::cout << ref << std::endl;
		return ref;
	}
};

class rs232_microbus_tcp_receiver
{
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Publisher pub_microbus_can_501_, pub_microbus_can_502_, pub_microbus_can_503_;
	socket_class socketclass;

	struct
	{
		bool read501, read502;
	} read_id_flag_;
public:
	rs232_microbus_tcp_receiver(ros::NodeHandle nh, ros::NodeHandle p_nh)
	{
		read_id_flag_.read501 = read_id_flag_.read502 = false;

		std::string ip = "192.168.1.10";
		int port = 100;

		socketclass.socket_connect(ip, port);
		if(socketclass.isConnect()) std::cout << "connect OK" << std::endl;

		pub_microbus_can_501_ = nh_.advertise<autoware_can_msgs::MicroBusCan501>("/microbus/can_receive501", 10);
		pub_microbus_can_502_ = nh_.advertise<autoware_can_msgs::MicroBusCan502>("/microbus/can_receive502", 10);
		pub_microbus_can_503_ = nh_.advertise<autoware_can_msgs::MicroBusCan503>("/microbus/can_receive503", 10);
	}

	void reader()
	{
		//socketclass.reader();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rs232_microbus_can_receiver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	rs232_microbus_tcp_receiver receiver(nh, private_nh);

	while(ros::ok())
	{
		receiver.reader();
		ros::spinOnce();
	}
	return 0;
}
