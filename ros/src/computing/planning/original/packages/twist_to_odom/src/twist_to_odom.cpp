#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;

void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  static double x = 0;
  static double y = 0;
  static double theta = 0;

  double current_time = msg->header.stamp.toSec();
  static double prev_time = current_time;
  const double dt = current_time - prev_time;
  prev_time = current_time;

  theta += msg->twist.angular.z * dt;

  const double dis = msg->twist.linear.x * dt;
  x += dis * std::cos(theta);
  y += dis * std::sin(theta);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "/odom";
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  odom_msg.twist.twist = msg->twist;
  odom_pub.publish(odom_msg);

  std::cout << "x:" << odom_msg.pose.pose.position.x << " "
            << "y:" << odom_msg.pose.pose.position.y << " "
            << "t:" << theta*180.0/M_PI << " "
            << "v:" << odom_msg.twist.twist.linear.x << " "
            << "w:" << odom_msg.twist.twist.angular.z << " "
            << std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc ,argv, "twist_to_odom");
  ros::NodeHandle nh;
  //odom_pub = nh.advertise<nav_msgs::Odometry>("/logiee/odom", 10);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/vehicle/odom", 10);
  ros::Subscriber sub = nh.subscribe("/vehicle/twist", 10, callbackTwist);
  //ros::Subscriber sub = nh.subscribe("/ymc_current_twist", 10, callbackTwist);

  //odom_msg.header.child_frame_id = "/base_link";

  ros::spin();

  return 0;
}
