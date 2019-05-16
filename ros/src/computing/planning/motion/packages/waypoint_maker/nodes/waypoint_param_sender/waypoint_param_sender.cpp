#include <ros/ros.h>
#include <autoware_msgs/LaneArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_msgs/AdjustXY.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_config_msgs/ConfigVoxelGridFilter.h>
#include <autoware_msgs/Signals.h>
#include <float.h>
#include <math.h>

ros::Publisher pub_waypoint_param;
ros::Publisher pub_adjust_xy;
ros::Publisher pub_voxelGridFilter;
ros::Publisher pub_roi_signal;

std::vector<geometry_msgs::Point> poseList;
std::vector<float> weightList;
std::vector<int> feat_proj_xList;
std::vector<int> feat_proj_yList;
std::vector<int> velocity_KPPlusList;
std::vector<int> velocity_KPMinusList;
std::vector<int> velocity_punchPlusList;
std::vector<int> velocity_punchMinusList;
std::vector<int> velocity_windowPlusList;
std::vector<int> velocity_windowMinusList;
std::vector<int> blinkerList;
std::vector<int> pauseList;
std::vector<float> vgf_leafsizeList;
std::vector<float> vgf_measurement_rangeList;
std::vector<std::vector<autoware_msgs::ExtractedPosition>> extracted_positionList;
std::vector<int> curve_flagList;

geometry_msgs::Pose local_point_sento;
float gnss_weight = 0;

void local_waypoints_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    static int prev_pause = 0;

    geometry_msgs::Point po;
    po.x = msg->markers[0].pose.position.x;
    po.y = msg->markers[0].pose.position.y;
    po.z = msg->markers[0].pose.position.z;

    int min_cou=0;
    float min_uc = FLT_MAX;
    float min_weight;
    int min_feat_proj_x, min_feat_proj_y;
    int min_blinker;
    int min_velocity_KPPlus, min_velocity_KPMinus;
    int min_velocity_punchPlus, min_velocity_punchMinus;
    int min_velocity_windowPlus, min_velocity_windowMinus;
    int min_pause;
    float min_vgf_leafsize, min_vgf_measurement_range;
    std::vector<autoware_msgs::ExtractedPosition> min_extracted_position;
    int min_curve;

    for(int cou=0; cou<poseList.size(); cou++)
    {
        float x = poseList[cou].x-po.x;
        float y = poseList[cou].y-po.y;
        float z = poseList[cou].z-po.z;
        float uc = sqrt(x*x + y*y + z*z);
        if(min_uc > uc)
        {
            min_cou = cou;
            min_uc = uc;
            min_weight = weightList[cou];
            min_feat_proj_x = feat_proj_xList[cou];
            min_feat_proj_y = feat_proj_yList[cou];
            min_blinker = blinkerList[cou];
            min_velocity_KPPlus = velocity_KPPlusList[cou];
            min_velocity_KPMinus = velocity_KPMinusList[cou];
            min_velocity_punchPlus = velocity_punchPlusList[cou];
            min_velocity_punchMinus = velocity_punchMinusList[cou];
            min_velocity_windowPlus = velocity_windowPlusList[cou];
            min_velocity_windowMinus = velocity_windowMinusList[cou];
            min_pause = pauseList[cou];
            min_vgf_leafsize = vgf_leafsizeList[cou];
            min_vgf_measurement_range = vgf_measurement_rangeList[cou];
            min_extracted_position = extracted_positionList[cou];
            min_curve = curve_flagList[cou];
        }
    }
    if(gnss_weight == min_weight)
    {
        ros::Time nowTime = ros::Time::now();

        autoware_msgs::WaypointParam param;
        param.header.frame_id = "map";
        param.header.stamp = nowTime;
        param.weight = min_weight;
        param.blinker = min_blinker;
        param.velocity_KPPlus = min_velocity_KPPlus;
        param.velocity_KPMinus = min_velocity_KPMinus;
        param.velocity_punchPlus = min_velocity_punchPlus;
        param.velocity_punchMinus = min_velocity_punchMinus;
        param.velocity_windowPlus = min_velocity_windowPlus;
        param.velocity_windowMinus = min_velocity_windowMinus;
        param.curve_flag = min_curve;
        if(prev_pause == min_pause)
        {std::cout<<"0 : "<<min_pause<<","<<prev_pause<<std::endl;
            param.pause = 0;
            std::cout<<"x:"<<min_feat_proj_x<<std::endl;
            std::cout<<"y:"<<min_feat_proj_y<<std::endl;
        }
        else
        {std::cout<<"1 : "<<min_pause<<","<<prev_pause<<std::endl;
            param.pause = min_pause;
            prev_pause = min_pause;
        }
        pub_waypoint_param.publish(param);

        if(min_feat_proj_x > -10000)
        {
            autoware_msgs::AdjustXY ad_xy;
            ad_xy.header.frame_id = "";
            ad_xy.header.stamp = nowTime;
            ad_xy.header.seq = min_cou;
            ad_xy.x = min_feat_proj_x;
            ad_xy.y = min_feat_proj_y;
            pub_adjust_xy.publish(ad_xy);
        }

        if(min_vgf_leafsize > 0)
        {
            autoware_config_msgs::ConfigVoxelGridFilter cvgf;
            cvgf.voxel_leaf_size = min_vgf_leafsize;
            cvgf.measurement_range = min_vgf_measurement_range;
            pub_voxelGridFilter.publish(cvgf);
        }

        if(min_extracted_position.size() == 3)
        {
            autoware_msgs::Signals sig_msg;
            sig_msg.header.frame_id = "";
            sig_msg.header.stamp = ros::Time::now();
            sig_msg.header.seq = 0;
            for(int i=0;i<3;i++)
            {
                autoware_msgs::ExtractedPosition ep;
                ep.signalId = min_extracted_position[i].signalId;
                ep.u = min_extracted_position[i].u;
                ep.v = min_extracted_position[i].v;
                ep.radius = min_extracted_position[i].radius;
                ep.x = min_extracted_position[i].x;
                ep.y = min_extracted_position[i].y;
                ep.z = min_extracted_position[i].z;
                ep.hang = min_extracted_position[i].hang;
                ep.type = min_extracted_position[i].type;
                ep.linkId = min_extracted_position[i].linkId;
                ep.plId = min_extracted_position[i].plId;
                sig_msg.Signals.push_back(ep);
            }
            pub_roi_signal.publish(sig_msg);
        }
    }
    gnss_weight = min_weight;
    //std::cout<<gnss_weight<<std::endl;
    //std::cout<<local_point_sento.position.x<<std::endl;
    //std::cout<<local_point_sento.position.y<<std::endl;
    //std::cout<<local_point_sento.position.z<<std::endl;
}

void lane_waypoints_array_callback(const autoware_msgs::LaneArray::ConstPtr& msg)
{
    float min_uc = FLT_MAX;
    float min_weight;

    poseList.clear();
    weightList.clear();
    feat_proj_xList.clear();
    feat_proj_yList.clear();
    blinkerList.clear();
    velocity_KPPlusList.clear();
    velocity_KPMinusList.clear();
    velocity_punchPlusList.clear();
    velocity_punchMinusList.clear();
    velocity_windowPlusList.clear();
    velocity_windowMinusList.clear();
    pauseList.clear();
    vgf_leafsizeList.clear();
    vgf_measurement_rangeList.clear();
    extracted_positionList.clear();

    //for(int lanesCou=0; lanesCou<msg->lanes.size(); lanesCou++)
    for(autoware_msgs::Lane lane : msg->lanes)
    {
        for(autoware_msgs::Waypoint waypoint : lane.waypoints)
        {
            geometry_msgs::Point po;
            po.x = waypoint.pose.pose.position.x;
            po.y = waypoint.pose.pose.position.y;
            po.z = waypoint.pose.pose.position.z;
            poseList.push_back(po);
            weightList.push_back(waypoint.waypoint_param.weight);
            feat_proj_xList.push_back(waypoint.waypoint_param.feat_proj_x);
            feat_proj_yList.push_back(waypoint.waypoint_param.feat_proj_y);
            blinkerList.push_back(waypoint.waypoint_param.blinker);
            velocity_KPPlusList.push_back(waypoint.waypoint_param.velocity_KPPlus);
            velocity_KPMinusList.push_back(waypoint.waypoint_param.velocity_KPMinus);
            velocity_punchPlusList.push_back(waypoint.waypoint_param.velocity_punchPlus);
            velocity_punchMinusList.push_back(waypoint.waypoint_param.velocity_punchMinus);
            velocity_windowPlusList.push_back(waypoint.waypoint_param.velocity_windowPlus);
            velocity_windowMinusList.push_back(waypoint.waypoint_param.velocity_windowMinus);
            pauseList.push_back(waypoint.waypoint_param.pause);
            vgf_leafsizeList.push_back(waypoint.waypoint_param.vgf_leafsize);
            vgf_measurement_rangeList.push_back(waypoint.waypoint_param.vgf_measurement_range);
            curve_flagList.push_back(waypoint.waypoint_param.curve_flag);

            std::vector<autoware_msgs::ExtractedPosition> eplist;
            for(int cou=0; cou<waypoint.waypoint_param.signals.size(); cou++)
            {
                autoware_msgs::ExtractedPosition ep;
                ep.signalId = waypoint.waypoint_param.signals[cou].signalId;
                ep.u = waypoint.waypoint_param.signals[cou].u;
                ep.v = waypoint.waypoint_param.signals[cou].v;
                ep.radius = waypoint.waypoint_param.signals[cou].radius;
                ep.x = waypoint.waypoint_param.signals[cou].x;
                ep.y = waypoint.waypoint_param.signals[cou].y;
                ep.z = waypoint.waypoint_param.signals[cou].z;
                ep.hang = waypoint.waypoint_param.signals[cou].hang;
                ep.type = waypoint.waypoint_param.signals[cou].type;
                ep.linkId = waypoint.waypoint_param.signals[cou].linkId;
                ep.plId = waypoint.waypoint_param.signals[cou].plId;
                eplist.push_back(ep);
            }
            extracted_positionList.push_back(eplist);
        }
    }
    //gnss_weight = min_weight;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_param_sender");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    pub_waypoint_param    = nh.advertise<autoware_msgs::WaypointParam>("/waypoint_param",10);
    pub_adjust_xy = nh.advertise<autoware_msgs::AdjustXY>("/config/adjust_xy",10);
    pub_voxelGridFilter = nh.advertise<autoware_config_msgs::ConfigVoxelGridFilter>("/config/voxel_grid_filter",10);
    pub_roi_signal = nh.advertise<autoware_msgs::Signals>("/loader_roi_signal",10);
    ros::Subscriber sub_lane_waypoints_array = nh.subscribe<autoware_msgs::LaneArray>("/lane_waypoints_array",10,lane_waypoints_array_callback);
    ros::Subscriber sub_local_waypoints = nh.subscribe<visualization_msgs::MarkerArray>("/local_waypoints_mark",10,local_waypoints_callback);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
