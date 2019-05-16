#ifndef WAYPOINT_FIELD_PATTERN
#define WAYPOINT_FIELD_PATTERN

//add msg type header file
#include <autoware_msgs/AdjustXY.h>

struct WaypointFieldPattern
{
    enum TypeList //add msg type definition
    {
        WFPID_ADJUST_XY_X = 1,
        WFPID_ADJUST_XY_Y = 2,
    };

    std::string field_name_;
    TypeList type_id_;
};

#endif
