#include "waypoint_param_sender2.h"

WaypointParamSender::WaypointParamSender(ros::NodeHandle nh, ros::NodeHandle p_nh)
{
    nh_ = nh;  private_nh_ = p_nh;

    sub_safety_waypoints_ = nh_.subscribe<autoware_msgs::Lane>("/safety_waypoints",
        10, &WaypointParamSender::safety_waypoints_callback, this);
    sub_final_waypoints_ = nh_.subscribe<autoware_msgs::Lane>("/final_waypoints",
        10, &WaypointParamSender::final_waypoints_callback, this);
}

WaypointParamSender::~WaypointParamSender()
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_param_sender");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    WaypointParamSender wps(nh, private_nh);
    std::cout << "aaa\n";

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
