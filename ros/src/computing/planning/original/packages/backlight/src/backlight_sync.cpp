#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


std::vector<std::string> split(const std::string &string, char split_token);

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nmea_msgs::Sentence>
    ImagePoseSync;

class SyncImageSaver
{
public:
	SyncImageSaver()
		: nh_()
		, private_nh_("~")
	{

		message_filters::Subscriber<sensor_msgs::Image> image_sub_(nh_, "/image_raw", 50);
  		message_filters::Subscriber<nmea_msgs::Sentence> nmea_sub_(nh_, "/nmea_sentence", 50);
  		message_filters::Synchronizer<ImagePoseSync> sync_tp_(ImagePoseSync(50), image_sub_, nmea_sub_);
   		sync_tp_.registerCallback(boost::bind(&SyncImageSaver::SyncImageCallback, this, _1, _2));
		ros::spin();
	}
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	
	
	void SyncImageCallback(const sensor_msgs::Image::ConstPtr& msg_img,const nmea_msgs::Sentence::ConstPtr& msg_nmea)
	{
	std::string timea; 
	std::vector<std::string> split_sentence = split(msg_nmea->sentence, ',');
		if(split_sentence[0]=="#TIMEA")
		{
		std::string str = msg_nmea->sentence;
		timea = str.substr(0,str.find_last_of(','));
		std::cout << timea << std::endl;
	
		cv_bridge::CvImagePtr cv_ptr;
        	cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
			cv::imwrite("/home/autoware/load_data/honjo/signal_time/" + timea + ".png",cv_ptr->image);

		}
	}
	//std::cout << input->sentence << std::endl;
};

int main(int argc,char *argv[])
{
	////ROS STUFF////

	ros::init(argc, argv, "backlight");
	SyncImageSaver SIS;

	return 0;
}

std::vector<std::string> split(const std::string &string, char split_token)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, split_token))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

