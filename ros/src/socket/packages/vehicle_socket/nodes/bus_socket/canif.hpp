#ifndef ESTIMA_CAN_INTERFACE_CANIF_HPP
#define ESTIMA_CAN_INTERFACE_CANIF_HPP

#include <string>
#include <linux/can.h>

class AutowareCanInterface
{
    public:

        bool init(const std::string& device);
        void dump(const can_frame& frame);
        std::string error();

        ssize_t send(const can_frame& frame);
        ssize_t receive(can_frame* frame);

    protected:

        int fd_;
        std::string error_;
};

#endif
