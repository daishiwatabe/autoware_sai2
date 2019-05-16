#ifndef WAYPOINT_PARAM_SENDER
#define WAYPOINT_PARAM_SENDER

#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <std_msgs/Int32.h>
#include "waypoint_field_pattern.h"

class WaypointParamSender
{
private:
    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber sub_safety_waypoints_;
    ros::Subscriber sub_final_waypoints_;

    autoware_msgs::Waypoint closest_waypoint_;

    void safety_waypoints_callback(const autoware_msgs::Lane::ConstPtr& msg);
    void final_waypoints_callback(const autoware_msgs::Lane::ConstPtr& msg);
public:
    WaypointParamSender(ros::NodeHandle nh, ros::NodeHandle p_nh);
    ~WaypointParamSender();
};

#endif
