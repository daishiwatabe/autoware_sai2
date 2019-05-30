#include "estima.hpp"
#include <cstdio>

constexpr double TIMESTAMP_RESOLUTION = 0.001;
constexpr double STEERING_RESOLUTION = 0.1;
constexpr double STEERING_RATIO_OFFSET = 10.0;
constexpr double STEERING_RATIO_RESOLUTION = 0.1;
constexpr double VELOCITY_RESOLUTION = 0.01;

void frame_set(uint8_t* data, int pos, int size, uint32_t value)
{
    while(--size >= 0)
    {
        data[pos + size] = 0xFF & value;
        value >>= 8;
    }
}

uint32_t frame_get(const uint8_t* data, int pos, int size)
{
    uint32_t value = 0;
    for(int i = 0; i < size; ++i)
    {
        value <<= 8;
        value |= data[pos + i];
    }
    return value;
}

std::string tostring_estima_can_message(const EstimaCanData& data)
{
    char buffer[4096];
    int pos = 0;
    pos += sprintf(buffer + pos, "Automode  %02x   Auto:%d Str:%d Vel:%d\n", data.autodrive_raw_field, data.autodrive_enabled, data.autodrive_steering, data.autodrive_velocity);
    pos += sprintf(buffer + pos, "Blinker   %02x   Auto:%d Left:%d Right:%d\n", data.blinker_raw_field, data.blinker_control, data.blinker_left, data.blinker_right);
    pos += sprintf(buffer + pos, "Gearshift %02x   Auto:%d Value:%d\n", data.gearshift_raw_field, data.gearshift_control, data.gearshift_request);
    pos += sprintf(buffer + pos, "Steering  %08x %04x %02x %02x", data.steering_raw_time, data.steering_raw_val, data.steering_raw_msgcnt, data.steering_raw_ratio);
    pos += sprintf(buffer + pos, "   Time:%.3f Deg:%.3f Ratio:%.2f\n", data.steering_time, data.steering_deg, data.steering_ratio);
    pos += sprintf(buffer + pos, "Velocity  %08x %04x %02x   ", data.velocity_raw_time, data.velocity_raw_val, data.velocity_raw_msgcnt);
    pos += sprintf(buffer + pos, "   Time:%.3f Kph:%.3f\n", data.velocity_time, data.velocity_kph);
    return std::string(buffer);
}

can_frame make_steering_frame(EstimaCanData& data)
{
    data.steering_raw_time = static_cast<uint32_t>(data.steering_time / TIMESTAMP_RESOLUTION);
    if(data.steering_deg < 0)
    {
        uint16_t abs_val = static_cast<uint16_t>(-data.steering_deg / STEERING_RESOLUTION);
        data.steering_raw_val = static_cast<uint16_t>(0x10000 - abs_val);
    }
    else
    {
        data.steering_raw_val = static_cast<uint16_t>(+data.steering_deg / STEERING_RESOLUTION);
    }

    can_frame frame;
    frame.can_id  = 0x100;
    frame.can_dlc = 8;
    frame_set(frame.data, 0, 4, data.steering_raw_time);
    frame_set(frame.data, 4, 2, data.steering_raw_val);
    frame_set(frame.data, 6, 1, data.steering_raw_msgcnt);
    frame_set(frame.data, 7, 1, 0);
    return frame;
}

can_frame make_velocity_frame(EstimaCanData& data)
{
    data.velocity_raw_time = static_cast<uint32_t>(data.velocity_time / TIMESTAMP_RESOLUTION);
    data.velocity_raw_val  = static_cast<uint16_t>(data.velocity_kph / VELOCITY_RESOLUTION);

    can_frame frame;
    frame.can_id  = 0x101;
    frame.can_dlc = 8;
    frame_set(frame.data, 0, 4, data.velocity_raw_time);
    frame_set(frame.data, 4, 2, data.velocity_raw_val);
    frame_set(frame.data, 6, 1, data.velocity_raw_msgcnt);
    frame_set(frame.data, 7, 1, 0);
    return frame;
}

can_frame make_bcm_frame(EstimaCanData& data)
{
    data.blinker_raw_field = 0;
    if(data.blinker_control ) { data.blinker_raw_field |= 0x80; }
    if(data.blinker_left    ) { data.blinker_raw_field |= 0x01; }
    if(data.blinker_right   ) { data.blinker_raw_field |= 0x02; }

    data.gearshift_raw_field = 0;
    if(data.gearshift_control)
    {
        data.gearshift_raw_field |= 0x80;
        data.gearshift_raw_field |= data.gearshift_request;
    }

    can_frame frame;
    frame.can_id  = 0x102;
    frame.can_dlc = 3;
    frame_set(frame.data, 0, 1, data.blinker_raw_field);
    frame_set(frame.data, 1, 1, data.gearshift_raw_field);
    frame_set(frame.data, 2, 1, 0);
    return frame;
}

can_frame make_autodrive_frame(EstimaCanData& data)
{
    data.autodrive_raw_field = 0;
    if(data.autodrive_enabled ) { data.autodrive_raw_field |= 0x80; }
    if(data.autodrive_steering) { data.autodrive_raw_field |= 0x40; }
    if(data.autodrive_velocity) { data.autodrive_raw_field |= 0x20; }

    can_frame frame;
    frame.can_id  = 0x103;
    frame.can_dlc = 2;
    frame_set(frame.data, 0, 1, data.autodrive_raw_field);
    frame_set(frame.data, 1, 1, 0);
    return frame;
}

void load_steering_frame(EstimaCanData& data, const can_frame& frame)
{
    data.steering_raw_time   = frame_get(frame.data, 0, 4);
    data.steering_raw_val    = frame_get(frame.data, 4, 2);
    data.steering_raw_msgcnt = frame_get(frame.data, 6, 1);
    data.steering_raw_ratio  = frame_get(frame.data, 7, 1);

    data.steering_time = data.steering_raw_time * TIMESTAMP_RESOLUTION;
    if(data.steering_raw_val & 0x8000) // sign bit is 1
    {
        data.steering_deg = -(0x10000 - data.steering_raw_val) * STEERING_RESOLUTION;
    }
    else
    {
        data.steering_deg = data.steering_raw_val * STEERING_RESOLUTION;
    }
    data.steering_ratio = (data.steering_raw_ratio * STEERING_RATIO_RESOLUTION) + STEERING_RATIO_OFFSET;
}

void load_velocity_frame(EstimaCanData& data, const can_frame& frame)
{
    data.velocity_raw_time   = frame_get(frame.data, 0, 4);
    data.velocity_raw_val    = frame_get(frame.data, 4, 2);
    data.velocity_raw_msgcnt = frame_get(frame.data, 6, 1);

    data.velocity_time = data.velocity_raw_time * TIMESTAMP_RESOLUTION;
    data.velocity_kph  = data.velocity_raw_val  * VELOCITY_RESOLUTION;
}

void load_bcm_frame(EstimaCanData& data, const can_frame& frame)
{
    data.blinker_raw_field   = frame_get(frame.data, 0, 1);
    data.gearshift_raw_field = frame_get(frame.data, 1, 1);

    data.blinker_control = (data.blinker_raw_field & 0x80);
    data.blinker_left    = (data.blinker_raw_field & 0x01);
    data.blinker_right   = (data.blinker_raw_field & 0x02);
    data.gearshift_control = data.gearshift_raw_field & 0x80;
    data.gearshift_request = data.gearshift_raw_field & 0x7F;
}

void load_autodrive_frame(EstimaCanData& data, const can_frame& frame)
{
    data.autodrive_raw_field = frame_get(frame.data, 0, 1);

    data.autodrive_enabled  = data.autodrive_raw_field & 0x80;
    data.autodrive_steering = data.autodrive_raw_field & 0x40;
    data.autodrive_velocity = data.autodrive_raw_field & 0x20;
}

bool load_frame(EstimaCanData& data, const can_frame& frame)
{
    switch(frame.can_id)
    {
        case 0x200: load_steering_frame(data, frame);  return true;
        case 0x201: load_velocity_frame(data, frame);  return true;
        case 0x202: load_bcm_frame(data, frame);       return true;
        case 0x203: load_autodrive_frame(data, frame); return true;
    }
    return false;
}
