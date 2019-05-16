#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/PositionChecker.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class PositionChecker
{
private:
    ros::Subscriber sub_local_waypoint_;
    ros::Subscriber sub_current_pose_;

    ros::Publisher pub_position_checker_;

    int waypoint_max_count=0;
    geometry_msgs::Point way_first_, way_second_, way_third_;
    double limit_euc_, permission_angle_;
    geometry_msgs::Quaternion lane_Quat_;

    bool lane_callback_stop_flag_ = true;

    void local_waypoints_callback(const autoware_msgs::Lane::ConstPtr& msg)
    {
        autoware_msgs::PositionChecker checker;
        checker.header.frame_id = "map";
        checker.header.stamp = ros::Time();

        //std::cout << msg->waypoints.size() << std::endl;
        waypoint_max_count = msg->waypoints.size();
        if(waypoint_max_count < 3)
        {
            checker.stop_flag = true;
            pub_position_checker_.publish(checker);
            lane_callback_stop_flag_ = checker.stop_flag;
            return;
        }

        way_first_.x =  msg->waypoints[0].pose.pose.position.x;
        way_first_.y =  msg->waypoints[0].pose.pose.position.y;
        way_first_.z =  msg->waypoints[0].pose.pose.position.z;
        way_second_.x =  msg->waypoints[1].pose.pose.position.x;
        way_second_.y =  msg->waypoints[1].pose.pose.position.y;
        way_second_.z =  msg->waypoints[1].pose.pose.position.z;
        way_third_.x =  msg->waypoints[2].pose.pose.position.x;
        way_third_.y =  msg->waypoints[2].pose.pose.position.y;
        way_third_.z =  msg->waypoints[2].pose.pose.position.z;
        std::cout << "local_waypoint_0    x : " << way_first_.x << " y : " << way_first_.y << " z : "<< way_first_.z << std::endl;
        //std::cout << "local_waypoint_1,x," << msg->waypoints[1].pose.pose.position.x-way_first_.x << ",y," << msg->waypoints[1].pose.pose.position.y-way_first_.y << ",z, "<< msg->waypoints[1].pose.pose.position.z-way_first_.z << std::endl;
        double sx = fabs(way_first_.x-way_second_.x), sy = fabs(way_first_.y-way_second_.y), sz = fabs(way_first_.z-way_second_.z);

        if(sx > limit_euc_ || sy > limit_euc_ || sz > limit_euc_) checker.stop_flag = true;
        else checker.stop_flag = false;
        std::cout << "sx : " << sx << "  sy : " << sy << "  sz : " << sz << std::endl;
        pub_position_checker_.publish(checker);
        lane_callback_stop_flag_ = checker.stop_flag;

        //std::cout << way_first_.x << "," << way_first_.y << "," << way_first_.z << std::endl;
        double rot_line_x = way_third_.x - way_second_.x;
        double rot_line_y = way_third_.y - way_second_.y;
        double rot_line_z = way_third_.z - way_second_.z;
        double yaw = atan2(rot_line_y, rot_line_x);
        double pitch = 0;
        double roll = 0;
        lane_Quat_ = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        /*tf::Quaternion tmp(lane_Quat_.x,lane_Quat_.y,lane_Quat_.z,lane_Quat_.w);
        tf::Matrix3x3 m(tmp);
        double r,p,y;
        m.getRPY(r,p,y);
        std::cout << "yaw," << yaw << "," << y << std::endl;*/
    }

    void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if(lane_callback_stop_flag_ == true) return;

        autoware_msgs::PositionChecker checker;
        checker.header.frame_id = "map";
        checker.header.stamp = ros::Time();
        checker.stop_flag = false;

        if(waypoint_max_count < 3)
        {
            std::cout << "not local waypoint" << std::endl;
            checker.stop_flag = true;
            pub_position_checker_.publish(checker);
            return;
        }
        //std::cout << "current_pose    x : " << msg->pose.position.x << " y : " << msg->pose.position.y << " z : "<< msg->pose.position.z << std::endl;
        //std::cout << "x : " << x << " y : "<< y << " z : " << z << std::endl;
        //std::cout << "euc : " << euc << std::endl;

        tf::Quaternion qpose(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
        tf::Matrix3x3 mpose(qpose);
        double mpx,mpy,mpz;
        mpose.getRPY(mpx,mpy,mpz);
        tf::Quaternion qlane(lane_Quat_.x,lane_Quat_.y,lane_Quat_.z,lane_Quat_.w);
        tf::Matrix3x3 mlane(qlane);
        double mlx,mly,mlz;
        mlane.getRPY(mlx,mly,mlz);
        //std::cout << "yaw," << permission_angle_ << "," << 2*M_PI-permission_angle_ << "," << mpz << "," << mlz << "," << mpz - mlz << std::endl;

        /*double yaw_sa = mpz - mlz;
        if(yaw_sa < 0 ) yaw_sa += 2*M_PI;
        if(yaw_sa < M_PI)
        {
            if(yaw_sa > permission_angle_) checker.stop_flag = true;
        }
        else
        {
            if(yaw_sa < 2*M_PI-permission_angle_) checker.stop_flag = true;
        }*/
        double mlz_0 = mlz - mpz;
        while(mlz_0 < 0) {mlz_0 += 2*M_PI;}
        while(mlz_0 > 2*M_PI) {mlz_0 -= 2*M_PI;}
        if(mlz_0 <=M_PI)
        {
            if(mlz > permission_angle_) checker.stop_flag = true;
        }
        if(mlz_0 > M_PI)
        {
            if(mlz < 2*M_PI-permission_angle_) checker.stop_flag = true;
        }
        //std::cout << mlz_0 << "," << (int)lane_callback_stop_flag_ << std::endl;

        checker.stop_flag = lane_callback_stop_flag_;
        pub_position_checker_.publish(checker);
        //std::cout << yaw_sa << "," << 2*M_PI << "," << 2*M_PI-permission_angle_ << "," << (int)checker.stop_flag << std::endl;
        //std::cout << "yaw," << permission_angle_ << "," << 2*M_PI-permission_angle_ << "," << yaw_sa << "," << ((checker.stop_flag)?1:0) << std::endl;

    }
public:
    PositionChecker(ros::NodeHandle nh, ros::NodeHandle pnh,
                    const std::string local_waypoint, std::string current_pose,
                    double limit_euc,double permission_angle)
    {
        pub_position_checker_ = nh.advertise<autoware_msgs::PositionChecker>("/position_checker", 10);

        sub_local_waypoint_ = nh.subscribe<autoware_msgs::Lane>(local_waypoint, 10, &PositionChecker::local_waypoints_callback, this);
        sub_current_pose_ = nh.subscribe<geometry_msgs::PoseStamped>(current_pose, 10, &PositionChecker::current_pose_callback, this);
        limit_euc_ = limit_euc;  permission_angle_ = permission_angle*M_PI/180.0;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "positon_checker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double limit_euc, permission_angle;
    private_nh.getParam("limit_euc", limit_euc);
    private_nh.getParam("permission_angle", permission_angle);

    std::string topic_local_waypoint = "/final_waypoints";
    std::string topic_current_pose = "/current_pose";
    PositionChecker position_checker(nh, private_nh, topic_local_waypoint, topic_current_pose,
                                     limit_euc, permission_angle);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
