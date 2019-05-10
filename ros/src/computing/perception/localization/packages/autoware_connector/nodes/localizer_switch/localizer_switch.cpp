#include <ros/ros.h>
#include <istream>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_config_msgs/ConfigLocalizerSwitchFusion.h>

const int max_localizer_count = 2;
const int SYNC_FRAMES = 10;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, autoware_msgs::NDTStat>
    NdtlocalizerSync;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, autoware_msgs::GnssStandardDeviation>
    RTKlocalizerSync;

//typedef void (* Localizer_Publisher)(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped);

class LocalizerSwitch;

tf::TransformListener *tf_listener;

class TopicList
{
private:
    ros::NodeHandle nh_, private_nh_;
    tf::TransformBroadcaster trans_broad;

    message_filters::Subscriber<geometry_msgs::PoseStamped>             *base_link_pose_sub_;
    message_filters::Subscriber<geometry_msgs::TwistStamped>            *estimate_twist_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped>             *localizer_pose_sub_;
    message_filters::Subscriber<autoware_msgs::NDTStat>                 *ndt_status_sub_;
    message_filters::Subscriber<autoware_msgs::GnssStandardDeviation> *gnss_deviation_sub_;
    message_filters::Synchronizer<NdtlocalizerSync> *sync_ndt_;
    message_filters::Synchronizer<RTKlocalizerSync> *sync_RTK_;

    ros::Publisher pub_current_pose_, pub_twist_pose_, pub_localizer_pose_;

    //class ID
    int classID;
    //Use localizer topic group
    std::string base_link_pose_topic_, estimate_twist_topic_, localizer_pose_topic_;
    //localizer's approach
    int approach_;
    //status topic to use when approach is ndt
    std::string ndt_status_topic_;
    //status topic to use when approach is GNSS(RTK)
    std::string gnss_deviation_topic_;
    //fusion select flag
    int fusion_select_;

    //baselink data
    geometry_msgs::PoseStamped baselink_;
    //twist data
    geometry_msgs::TwistStamped twist_;
    //localizer data
    geometry_msgs::PoseStamped localizer_;

    void posedata_write(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
                        const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
                        const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg)
    {
        baselink_.header.frame_id = base_link_pose_msg->header.frame_id;
        baselink_.header.stamp = base_link_pose_msg->header.stamp;
        baselink_.header.seq = base_link_pose_msg->header.seq;
        baselink_.pose.position.x = base_link_pose_msg->pose.position.x;
        baselink_.pose.position.y = base_link_pose_msg->pose.position.y;
        baselink_.pose.position.z = base_link_pose_msg->pose.position.z;
        baselink_.pose.orientation.x = base_link_pose_msg->pose.orientation.x;
        baselink_.pose.orientation.y = base_link_pose_msg->pose.orientation.y;
        baselink_.pose.orientation.z = base_link_pose_msg->pose.orientation.z;
        baselink_.pose.orientation.w = base_link_pose_msg->pose.orientation.w;

        twist_.header.frame_id = estimate_twist_msg->header.frame_id;
        twist_.header.stamp = estimate_twist_msg->header.stamp;
        twist_.header.seq = estimate_twist_msg->header.seq;
        twist_.twist.linear.x = estimate_twist_msg->twist.linear.x;
        twist_.twist.linear.y = estimate_twist_msg->twist.linear.y;
        twist_.twist.linear.z = estimate_twist_msg->twist.linear.z;
        twist_.twist.angular.x = estimate_twist_msg->twist.angular.x;
        twist_.twist.angular.y = estimate_twist_msg->twist.angular.y;
        twist_.twist.angular.z = estimate_twist_msg->twist.angular.z;

        localizer_.header.frame_id = localizer_pose_msg->header.frame_id;
        localizer_.header.stamp = localizer_pose_msg->header.stamp;
        localizer_.header.seq = localizer_pose_msg->header.seq;
        localizer_.pose.position.x = localizer_pose_msg->pose.position.x;
        localizer_.pose.position.y = localizer_pose_msg->pose.position.y;
        localizer_.pose.position.z = localizer_pose_msg->pose.position.z;
        localizer_.pose.orientation.x = localizer_pose_msg->pose.orientation.x;
        localizer_.pose.orientation.y = localizer_pose_msg->pose.orientation.y;
        localizer_.pose.orientation.z = localizer_pose_msg->pose.orientation.z;
        localizer_.pose.orientation.w = localizer_pose_msg->pose.orientation.w;
    }

    void NdtlocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
                                    const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
                                    const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg,
                                    const autoware_msgs::NDTStatConstPtr &ndt_stConstPtratus_msg)
    {
        //std::cout << "aaa\n" << std::flush;
        posedata_write(base_link_pose_msg, estimate_twist_msg, localizer_pose_msg);

        switch(fusion_select_)
        {
        case 1:
            {
                pose_topic_publish();
                break;
            }
        }
    }

    void RTKlocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
                                    const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
                                    const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg,
                                    const autoware_msgs::GnssStandardDeviationConstPtr &gnss_deviation_msg)
    {
        //std::cout << "bbb\n" << std::flush;
        posedata_write(base_link_pose_msg, estimate_twist_msg, localizer_pose_msg);

        switch(fusion_select_)
        {
        case 1:
            {
                pose_topic_publish();
                break;
            }
        }
    }
public:
    TopicList(ros::NodeHandle nh, ros::NodeHandle private_nh,
              std::string baseLinkPoseTopic, std::string estimateTwistTopic, std::string localizerPoseTopic,
              int approachFlag, std::string ndtStatusTopic, std::string gnssDeviationTopic)
        : nh_(nh)
        , private_nh_(private_nh)
        , fusion_select_(-1)
    {
        static int classID_counter = 0;
        classID = classID_counter;   classID_counter++;
        base_link_pose_topic_ = baseLinkPoseTopic;   estimate_twist_topic_ = estimateTwistTopic;
        localizer_pose_topic_ = localizerPoseTopic;
        approach_ = approachFlag;
        ndt_status_topic_ = ndtStatusTopic;
        gnss_deviation_topic_ = gnssDeviationTopic;

        pub_current_pose_   = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
        pub_twist_pose_     = nh_.advertise<geometry_msgs::TwistStamped>("/current_velocity", 10);
        pub_localizer_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
    }

    std::string get_base_link_pose_topic() {return base_link_pose_topic_; }
    std::string get_estimate_twist_topic() {return estimate_twist_topic_; }
    std::string get_localizer_pose_topic() {return localizer_pose_topic_; }
    int         get_approach() {return approach_; }
    std::string get_ndt_status_topic() {return ndt_status_topic_; }
    std::string get_gnss_deviation_topic() {return gnss_deviation_topic_; }

    void callback_run()
    {
        // subscriber
        base_link_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, base_link_pose_topic_, 10);
        estimate_twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, estimate_twist_topic_, 10);
        localizer_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, localizer_pose_topic_, 10);
        switch(approach_)
        {
        case 0://ndt
            {
                ndt_status_sub_ = new message_filters::Subscriber<autoware_msgs::NDTStat>(nh_, ndt_status_topic_, 10);
                sync_ndt_ = new message_filters::Synchronizer<NdtlocalizerSync>(NdtlocalizerSync(SYNC_FRAMES),
                                  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_, *ndt_status_sub_);
                sync_ndt_->registerCallback(boost::bind(&TopicList::NdtlocalizerCallback, this, _1, _2, _3, _4));
                break;
            }
        case 1://gnss(RTK)
            {
                gnss_deviation_sub_ = new message_filters::Subscriber<autoware_msgs::GnssStandardDeviation>(nh_, gnss_deviation_topic_, 10);
                sync_RTK_ = new message_filters::Synchronizer<RTKlocalizerSync>(RTKlocalizerSync(SYNC_FRAMES),
                                  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_, *gnss_deviation_sub_);
                sync_RTK_->registerCallback(boost::bind(&TopicList::RTKlocalizerCallback, this, _1, _2, _3, _4));
                break;
            }
        }
    }

    void pose_topic_publish()
    {
        //std::cout << "aaa\n" << std::flush;
        geometry_msgs::PoseStamped base_link_pose_msg;
        base_link_pose_msg.header.frame_id = "base_link";
        base_link_pose_msg.header.stamp = baselink_.header.stamp;
        base_link_pose_msg.header.seq = baselink_.header.seq;
        base_link_pose_msg.pose.position.x = baselink_.pose.position.x;
        base_link_pose_msg.pose.position.y = baselink_.pose.position.y;
        base_link_pose_msg.pose.position.z = baselink_.pose.position.z;
        base_link_pose_msg.pose.orientation.x = baselink_.pose.orientation.x;
        base_link_pose_msg.pose.orientation.y = baselink_.pose.orientation.y;
        base_link_pose_msg.pose.orientation.z = baselink_.pose.orientation.z;
        base_link_pose_msg.pose.orientation.w = baselink_.pose.orientation.w;

        tf::StampedTransform tf_stamped;
        ros::Time time = ros::Time::now();
        bool flag;
        flag = tf_listener->waitForTransform("map", twist_.header.frame_id, twist_.header.stamp, ros::Duration(3.0));
        //if(flag == true) std::cout << "bbb\n" << std::flush;
        //else std::cout << "ccc\n" << std::flush;
        tf_listener->lookupTransform("map", twist_.header.frame_id, twist_.header.stamp, tf_stamped);
        //if(flag == false) std::cout << "bbb\n";
        trans_broad.sendTransform(tf::StampedTransform(tf_stamped, twist_.header.stamp, "/map", "/base_link"));

        geometry_msgs::TwistStamped twist_pose_msg;
        twist_pose_msg.header.frame_id = "base_link";
        twist_pose_msg.header.stamp = twist_.header.stamp;
        twist_pose_msg.header.seq = twist_.header.seq;
        twist_pose_msg.twist.linear.x = twist_.twist.linear.x;
        twist_pose_msg.twist.linear.y = twist_.twist.linear.y;
        twist_pose_msg.twist.linear.z = twist_.twist.linear.z;
        twist_pose_msg.twist.angular.x = twist_.twist.angular.x;
        twist_pose_msg.twist.angular.y = twist_.twist.angular.y;
        twist_pose_msg.twist.angular.z = twist_.twist.angular.z;

        geometry_msgs::PoseStamped localizer_pose_msg;
        localizer_pose_msg.header.frame_id = "base_link";
        localizer_pose_msg.header.stamp = localizer_.header.stamp;
        localizer_pose_msg.header.seq = localizer_.header.seq;
        localizer_pose_msg.pose.position.x = localizer_.pose.position.x;
        localizer_pose_msg.pose.position.y = localizer_.pose.position.y;
        localizer_pose_msg.pose.position.z = localizer_.pose.position.z;
        localizer_pose_msg.pose.orientation.x = localizer_.pose.orientation.x;
        localizer_pose_msg.pose.orientation.y = localizer_.pose.orientation.y;
        localizer_pose_msg.pose.orientation.z = localizer_.pose.orientation.z;
        localizer_pose_msg.pose.orientation.w = localizer_.pose.orientation.w;

        //tf_Broadcaster.sendTransform(tf::StampedTransform(tf_stamped, twist_.header.stamp, "/map", "/base_link"));
        pub_current_pose_.publish(baselink_);
        pub_twist_pose_.publish(twist_pose_msg);
        pub_localizer_pose_.publish(localizer_);
    }

    void set_fusion_select(int select)
    {
        fusion_select_ = select;
    }
};

class LocalizerSwitch
{
private:
    ros::NodeHandle nh_, private_nh_;

    std::vector<TopicList> topic_list_;

    ros::Subscriber sub_fusion_select_;


    tf::TransformBroadcaster tf_Broadcaster_;

    void fusion_select_callback(const autoware_config_msgs::ConfigLocalizerSwitchFusionConstPtr &msg)
    {
        std::cout << "fusuion select : " << msg->fusion_select << std::flush << std::endl;

        switch(msg->fusion_select)
        {
        case 0://pose1 only
            {
                for(int i=0; i<topic_list_.size(); i++)
                {
                    if(i == 0) topic_list_[0].set_fusion_select(1);
                    else topic_list_[i].set_fusion_select(0);
                }

                break;
            }
        case 1://pose2 only
            {
                for(int i=0; i<topic_list_.size(); i++)
                {
                    if(i == 1) topic_list_[1].set_fusion_select(1);
                    else topic_list_[i].set_fusion_select(0);
                }
                break;
            }
        }
    }

public:
    LocalizerSwitch(ros::NodeHandle nh, ros::NodeHandle private_nh, std::vector<TopicList> list)
    {
        nh_ = nh;  private_nh_ = private_nh;
        sub_fusion_select_ = nh_.subscribe<autoware_config_msgs::ConfigLocalizerSwitchFusion>(
                    "/config/fusion_select", 10, &LocalizerSwitch::fusion_select_callback, this);
        topic_list_ = list;
        for(int i=0; i<topic_list_.size(); i++)
        {
            topic_list_[i].callback_run();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_matching");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    tf_listener = new tf::TransformListener;

    std::vector<TopicList> topicListArray;

    for(int cou=1; cou<=max_localizer_count; cou++)
    {
        std::string base_link_pose_topic, estimate_twist_topic, localizer_pose_topic;
        std::stringstream base_link_pose_name, estimate_twist_name, localizer_pose_name, base_link_tf_name, alignment_mechanism_name;

        base_link_pose_name << "base_link_pose" << cou;
        private_nh.param<std::string>(base_link_pose_name.str(), base_link_pose_topic, std::string(""));
        std::cout << base_link_pose_name.str() << " : " << base_link_pose_topic << std::endl;

        estimate_twist_name << "estimate_twist" << cou;
        private_nh.param<std::string>(estimate_twist_name.str(), estimate_twist_topic, std::string(""));
        std::cout << estimate_twist_name.str() << " : " << estimate_twist_topic << std::endl;

        localizer_pose_name << "localizer_pose" << cou;
        private_nh.param<std::string>(localizer_pose_name.str(), localizer_pose_topic, std::string(""));
        std::cout << localizer_pose_name.str() << " : " << localizer_pose_topic << std::endl;

        alignment_mechanism_name << "alignment_mechanism" << cou;
        int alignment_mechanism;
        private_nh.param<int>(alignment_mechanism_name.str(), alignment_mechanism, 0);
        std::cout << alignment_mechanism_name.str() << " : " << alignment_mechanism << std::endl;

        std::string ndt_status_topic="", gnss_deviation_topic="";
        std::stringstream ndt_status_name, gnss_deviation_name;

        switch(alignment_mechanism)
        {
        case 0://ndt
            {
                ndt_status_name << "ndt_status" << cou;
                private_nh.param<std::string>(ndt_status_name.str(), ndt_status_topic, std::string(""));
                std::cout << ndt_status_name.str() << " : " << ndt_status_topic << std::endl;
                break;
            }
        case 1://GNSS(RTK)
            {
                gnss_deviation_name << "gnss_deviation" << cou;
                private_nh.param<std::string>(gnss_deviation_name.str(), gnss_deviation_topic, std::string(""));
                std::cout << gnss_deviation_name.str() << " : " << gnss_deviation_topic << std::endl;
                break;
            }
        }

        TopicList list(nh, private_nh,
                       base_link_pose_topic, estimate_twist_topic, localizer_pose_topic,
                       alignment_mechanism, ndt_status_topic, gnss_deviation_topic);
        topicListArray.push_back(list);
    }


    LocalizerSwitch localizer_switch(nh, private_nh, topicListArray);
    while(ros::ok())
    {
        ros::spinOnce();
    }

    delete tf_listener;
    return 0;
}
