#include "canif.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

bool AutowareCanInterface::init(const std::string& device)
{
	struct sockaddr_can addr;
	struct ifreq ifr;

	if((fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
		error_ = "failed to open socket";
		return false;
	}
	int flags = fcntl(fd_, F_GETFL, 0);
	fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

	strcpy(ifr.ifr_name, device.c_str());
	ioctl(fd_, SIOCGIFINDEX, &ifr);

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if(bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		error_ = "failed to bind socket " + device;
		return false;
	}

	error_ = std::string() + "connect to " + device + " at index " + std::to_string(ifr.ifr_ifindex);
    return true;
}

void AutowareCanInterface::dump(const can_frame& frame)
{
	printf("%4u [%u]", frame.can_id, frame.can_dlc);
	for (int i = 0; i < frame.can_dlc; i++)
	{
		printf(" %02x", frame.data[i]);
	}
	printf("\n");
}

std::string AutowareCanInterface::error()
{
	return error_;
}

ssize_t AutowareCanInterface::send(const struct can_frame& frame)
{
    return write(fd_, &frame, sizeof(struct can_frame));
}

ssize_t AutowareCanInterface::receive(struct can_frame* pframe)
{
    return read(fd_, pframe, sizeof(struct can_frame));
}