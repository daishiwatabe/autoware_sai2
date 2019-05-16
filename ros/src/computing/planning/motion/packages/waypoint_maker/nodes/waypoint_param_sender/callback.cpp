#include "waypoint_param_sender2.h"

void WaypointParamSender::safety_waypoints_callback(const autoware_msgs::Lane::ConstPtr& msg)
{
    int size = msg->waypoints.size();
    if(size <= 0)
    {
        std::cerr << "WaypointParamSender WARNING : Safety_waypoints size 0" << std::endl;
        return;
    }

    closest_waypoint_ = msg->waypoints[0];
    //std::cout << closest_waypoint_.feat_proj_x << std::endl;
}

void WaypointParamSender::final_waypoints_callback(const autoware_msgs::Lane::ConstPtr& msg)
{
    //std::cout << "read global waypoint" << std::endl;

    //if(msg->waypoints.size() <= 0) { std::cerr << "error : final waypoint way sizs is 0" << std::endl; return;}
    //std::cout << "feat : " << msg->waypoints[0].feat_proj_x << std::endl;


}
