#ifndef ESTIMA_CAN_INTERFACE_ESTIMA_HPP
#define ESTIMA_CAN_INTERFACE_ESTIMA_HPP

#include <cstdint>
#include <string>
#include <linux/can.h>

struct EstimaCanData
{
    bool     autodrive_enabled;
    bool     autodrive_steering;
    bool     autodrive_velocity;
    uint8_t  autodrive_raw_field;

    double   steering_time;
    double   steering_deg;
    double   steering_ratio;
    uint32_t steering_raw_time;
    uint16_t steering_raw_val;
    uint8_t  steering_raw_msgcnt;
    uint8_t  steering_raw_ratio;

    double   velocity_time;
    double   velocity_kph;
    uint32_t velocity_raw_time;
    uint16_t velocity_raw_val;
    uint8_t  velocity_raw_msgcnt;

    bool     blinker_control;
    bool     blinker_right;
    bool     blinker_left;
    uint8_t  blinker_raw_field;

    bool     gearshift_control;
    int      gearshift_request;
    uint8_t  gearshift_raw_field;
};

std::string tostring_estima_can_message(const EstimaCanData& data);

can_frame make_steering_frame(EstimaCanData& data);
can_frame make_velocity_frame(EstimaCanData& data);
can_frame make_bcm_frame(EstimaCanData& data);
can_frame make_autodrive_frame(EstimaCanData& data);

bool load_frame(EstimaCanData& data, const can_frame& frame);

void load_steering_frame(EstimaCanData& data, const can_frame& frame);
void load_velocity_frame(EstimaCanData& data, const can_frame& frame);
void load_bcm_frame(EstimaCanData& data, const can_frame& frame);
void load_autodrive_frame(EstimaCanData& data, const can_frame& frame);

#endif
