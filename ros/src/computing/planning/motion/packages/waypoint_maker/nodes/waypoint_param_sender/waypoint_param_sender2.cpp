#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/AdjustXY.h>
#include <autoware_config_msgs/ConfigVoxelGridFilter.h>
#include <autoware_msgs/Signals.h>
#include <autoware_config_msgs/ConfigLocalizerSwitchFusion.h>

class WaypointMaker
{
private:
	ros::NodeHandle nh_, private_nh_;

	ros::Subscriber sub_local_waypoint_;
	ros::Publisher pub_waypoint_param_, pub_adjust_xy_, pub_voxelGridFilter_, pub_roi_signal_, pub_fusion_select_;

	void callback_local_waypoints(const autoware_msgs::Lane& msg)
	{
		if(msg.waypoints.size() < 2) return;

		autoware_msgs::WaypointParam param = msg.waypoints[1].waypoint_param;
		pub_waypoint_param_.publish(param);

		if(param.feat_proj_x > -10000 && param.feat_proj_y > -10000)
		{
			autoware_msgs::AdjustXY ad_xy;
			ad_xy.header.frame_id = "";
			ad_xy.header.stamp = msg.header.stamp;
			ad_xy.header.seq = msg.header.seq;
			ad_xy.x = param.feat_proj_x;
			ad_xy.y = param.feat_proj_y;
			pub_adjust_xy_.publish(ad_xy);
		}

		if(param.vgf_leafsize > 0)
		{
			autoware_config_msgs::ConfigVoxelGridFilter cvgf;
			cvgf.voxel_leaf_size = param.vgf_leafsize;
			cvgf.measurement_range = param.vgf_measurement_range;
			pub_voxelGridFilter_.publish(cvgf);
		}

		if(param.signals.size() == 3)
		{
			autoware_msgs::Signals sig_msg;
			sig_msg.header.frame_id = "";
			sig_msg.header.stamp = ros::Time::now();
			sig_msg.header.seq = 0;
			for(int i=0;i<3;i++)
			{
				autoware_msgs::ExtractedPosition ep;
				ep.signalId = param.signals[i].signalId;
				ep.u = param.signals[i].u;
				ep.v = param.signals[i].v;
				ep.radius = param.signals[i].radius;
				ep.x = param.signals[i].x;
				ep.y = param.signals[i].y;
				ep.z = param.signals[i].z;
				ep.hang = param.signals[i].hang;
				ep.type = param.signals[i].type;
				ep.linkId = param.signals[i].linkId;
				ep.plId = param.signals[i].plId;
				sig_msg.Signals.push_back(ep);
			}
			pub_roi_signal_.publish(sig_msg);
		}

		if(param.fusion_select >= 0)
		{
			autoware_config_msgs::ConfigLocalizerSwitchFusion msg;
			msg.header.frame_id = "";
			msg.header.stamp = ros::Time::now();
			msg.header.seq = 0;
			msg.fusion_select = param.fusion_select;
			pub_fusion_select_.publish(msg);
		}
	}
public:
	WaypointMaker(ros::NodeHandle nh, ros::NodeHandle p_nh)
	{
		nh_ = nh;  private_nh_ = p_nh;

		pub_adjust_xy_ = nh.advertise<autoware_msgs::AdjustXY>("/config/adjust_xy",10);
		pub_voxelGridFilter_ = nh.advertise<autoware_config_msgs::ConfigVoxelGridFilter>("/config/voxel_grid_filter",10);
		pub_roi_signal_ = nh.advertise<autoware_msgs::Signals>("/loader_roi_signal",10);
		pub_waypoint_param_ = nh_.advertise<autoware_msgs::WaypointParam>("/waypoint_param", 1);
		pub_fusion_select_ = nh_.advertise<autoware_config_msgs::ConfigLocalizerSwitchFusion>("/config/fusion_select", 1);
		sub_local_waypoint_ = nh_.subscribe("/final_waypoints", 10, &WaypointMaker::callback_local_waypoints, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_param_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	WaypointMaker waypoint_maker(nh, private_nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
