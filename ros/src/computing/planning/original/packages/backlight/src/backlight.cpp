#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
static std::string timea;
static std::string insattxa;
static std::string bestpos;
std::vector<std::string> split(const std::string &string, char split_token);

void image_callback(const sensor_msgs::Image::ConstPtr& input)
{
        std::cout <<"aaa "<< input->height << "  " << input->width << std::endl;
	  cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
        cv::imwrite(timea + ".png",cv_ptr->image);
}
void nmea_callback(const nmea_msgs::Sentence::ConstPtr& input)
{

	std::vector<std::string> split_sentence = split(input->sentence, ',');
	if(split_sentence[0]=="#TIMEA")
	{
		std::string str = input->sentence;
		timea = str.substr(0,str.find_last_of(','));
		std::cout << timea << std::endl;
	}
	//std::cout << input->sentence << std::endl;
}

int main(int argc,char *argv[])
{
	////ROS STUFF////
	ros::init(argc, argv, "backlight");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ros::Subscriber sub_nmea = nh.subscribe("nmea_sentence", 10, nmea_callback);
	ros::Subscriber sub_img  = nh.subscribe("image_raw",10,image_callback);

	while (ros::ok())
	{
		ros::spinOnce();
	}  
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

