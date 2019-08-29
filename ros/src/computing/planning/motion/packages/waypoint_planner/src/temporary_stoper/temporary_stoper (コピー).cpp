#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <autoware_config_msgs/ConfigTemporaryStoper.h>

class TemporaryStoper
{
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_waypoint_;
	ros::Publisher pub_waypoint_;

	autoware_config_msgs::ConfigTemporaryStoper config_;

	uint32_t stop_waypoint_id_;
	ros::Time stop_timer_;

	autoware_msgs::Lane apply_acceleration(const autoware_msgs::Lane& lane, double acceleration, int start_index,
	                                       size_t fixed_cnt, double fixed_vel)
	{
	  autoware_msgs::Lane l = lane;

	  if (fixed_cnt == 0)
		return l;

	  double square_vel = fixed_vel * fixed_vel;
	  double distance = 0;
	  for (int i = start_index; i < l.waypoints.size(); ++i)
	  {
		if (i - start_index < fixed_cnt)
		{
			l.waypoints[i].twist.twist.linear.x = fixed_vel;
			continue;
		}

		geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
		geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
		distance += hypot(b.x - a.x, b.y - a.y);

		double v = sqrt(square_vel + 2 * acceleration * distance);
		if (v < l.waypoints[i].twist.twist.linear.x)
			l.waypoints[i].twist.twist.linear.x = v;
		else
			break;
	  }

	  return l;
	}

	int stop_line_search(const autoware_msgs::Lane& way)
	{
		for(int i=0; i<way.waypoints.size() || i<config_.search_distance; i++)
		{
			if(way.waypoints[i].waypoint_param.temporary_stop_line > 0)
				return i;
		}
		return -1;
	}

	autoware_msgs::Lane apply_stopline_acceleration(const autoware_msgs::Lane& lane, double acceleration,
	                                                int ahead_cnt, int behind_cnt)
	{
		int stop_index = stop_line_search(lane);
		if(stop_index < 0)
		{
			pub_waypoint_.publish(lane);
			return lane;
		}

		ros::Time now_time = ros::Time::now();
		autoware_msgs::Waypoint stop_point = lane.waypoints[stop_index];
		if(stop_timer_ < now_time)
		{
			if(stop_waypoint_id_ == stop_point.waypoint_param.id)
				return lane;
			else
			{
				stop_waypoint_id_ = stop_point.waypoint_param.id;
				stop_timer_ = ros::Time(now_time.sec + 5, now_time.nsec);
			}
		}

		autoware_msgs::Lane l = apply_acceleration(lane, acceleration, stop_index, behind_cnt + 1, 0);
		std::reverse(l.waypoints.begin(), l.waypoints.end());
		int reverse_stop_index = l.waypoints.size() - stop_index - 1;
		l = apply_acceleration(l, acceleration, reverse_stop_index, ahead_cnt + 1, 0);
		std::reverse(l.waypoints.begin(), l.waypoints.end());

		return l;
	}

	void callbackConfig(const autoware_config_msgs::ConfigTemporaryStoper& msg)
	{
		config_ = msg;
	}

	void callbackWaypoint(const autoware_msgs::Lane& msg)
	{
		autoware_msgs::Lane lane = apply_stopline_acceleration(msg, config_.acceleration,
		                            config_.number_of_zeros_ahead, config_.number_of_zeros_behind);
		pub_waypoint_.publish(lane);
	}
public:
	TemporaryStoper(ros::NodeHandle nh, ros::NodeHandle p_nh)
	    : stop_waypoint_id_(0)
	{
		nh_ = nh;  private_nh_ = p_nh;

		config_.search_distance = 30;
		config_.acceleration = 1;
		config_.number_of_zeros_ahead = 10;
		config_.number_of_zeros_behind = 10;

		stop_timer_ = ros::Time::now();

		pub_waypoint_ = nh_.advertise<autoware_msgs::Lane>("/temporary_stop_waypoints", 1);
		sub_waypoint_ = nh_.subscribe("/safety_waypoints", 1, &TemporaryStoper::callbackWaypoint, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "temporary_stoper");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	TemporaryStoper ts(nh, private_nh);
	while(ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}
