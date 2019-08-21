/*
 * signals.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */


#include <iostream>
#include <ros/ros.h>
#include "Rate.h"
#include "libvectormap/vector_map.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>
#include <cstdio>
#include "libvectormap/Math.h"
#include <Eigen/Eigen>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/AdjustXY.h>
#include <vector_map/vector_map.h>
#include <vector_map_server/GetSignal.h>
#include <autoware_msgs/Lane.h>

static std::string camera_id_str;

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static int adjust_proj_x = 0;
static int adjust_proj_y = 0;

typedef struct
{
	double thiX;
	double thiY;
	double thiZ;
} Angle;

//static Angle cameraOrientation; // camera orientation = car's orientation

static Eigen::Vector3f position;
static Eigen::Quaternionf orientation;
static float fx,
		fy,
		imageWidth,
		imageHeight,
		cx,
		cy;
static tf::StampedTransform trf;

static ros::ServiceClient g_ros_client;

#define SignalLampRadius 0.3

static autoware_msgs::Signals roi_signal;

void loader_roi_signal_Callback(const autoware_msgs::Signals::ConstPtr &sig_msg)
{
    roi_signal.Signals.clear();
    roi_signal.header.frame_id = "";
    roi_signal.header.stamp = ros::Time::now();
    roi_signal.header.seq = 0;
    if(sig_msg->Signals.size() != 3) return;


    for(int i=0;i<3;i++)
    {
        autoware_msgs::ExtractedPosition ep, msgep = sig_msg->Signals[i];
        ep.signalId = msgep.signalId;
        ep.u = msgep.u;
        ep.v = msgep.v;
        ep.radius = msgep.radius;
        ep.x = msgep.x;
        ep.y = msgep.y;
        ep.z = msgep.z;
        ep.hang = msgep.hang;
        ep.type = msgep.type;
        ep.linkId = msgep.linkId;
        ep.plId = msgep.plId; std::cout << ep.x << std::endl;
        roi_signal.Signals.push_back(ep);
    }
}

/* Callback function to shift projection result */
void adjust_xyCallback(const autoware_msgs::AdjustXY::ConstPtr &config_msg)
{
	adjust_proj_x = config_msg->x;
	adjust_proj_y = config_msg->y;
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr camInfoMsg)
{
	fx = static_cast<float>(camInfoMsg->P[0]);
	fy = static_cast<float>(camInfoMsg->P[5]);
	imageWidth = camInfoMsg->width;
	imageHeight = camInfoMsg->height;
	cx = static_cast<float>(camInfoMsg->P[2]);
	cy = static_cast<float>(camInfoMsg->P[6]);
}


/* convert degree value into 0 to 360 range */
/*static double setDegree0to360(double val)
{
	if (val < 0.0f)
	{
		return (val + 360.0f);
	} else if (360.0f < val)
	{
		return (val - 360.0f);
	}

	return val;
}*/


/*static void get_cameraRollPitchYaw(double *roll,
                                   double *pitch,
                                   double *yaw)
{
	geometry_msgs::Pose cameraPose;
	cameraPose.position.x = (double) (position.x());
	cameraPose.position.y = (double) (position.y());
	cameraPose.position.z = (double) (position.z());
	cameraPose.orientation.x = (double) (orientation.x());
	cameraPose.orientation.y = (double) (orientation.y());
	cameraPose.orientation.z = (double) (orientation.z());
	cameraPose.orientation.w = (double) (orientation.w());

	tf::Quaternion quat;

	tf::quaternionMsgToTF(cameraPose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);

	// convert from radian to degree
	*roll = setDegree0to360(*roll * 180.0f / M_PI);
	*pitch = setDegree0to360(*pitch * 180.0f / M_PI);
	*yaw = setDegree0to360(*yaw * 180.0f / M_PI);
}*/


/*
  check if lower < val < upper
  This function also considers circulation
*/
static bool isRange(const double lower, const double upper, const double val)
{
	if (lower <= upper)
	{
		if (lower < val && val < upper)
		{
			return true;
		}
	} else
	{
		if (val < upper || lower < val)
		{
			return true;
		}
	}

	return false;
}


void getTransform(Eigen::Quaternionf &ori, Point3 &pos)
{
	static tf::TransformListener listener;

	// target_frame    source_frame
	ros::Time now = ros::Time();
	listener.waitForTransform(camera_id_str, "map", now, ros::Duration(10.0));
	listener.lookupTransform(camera_id_str, "map", now, trf);

	tf::Vector3 &p = trf.getOrigin();
	tf::Quaternion o = trf.getRotation();
	pos.x() = p.x();
	pos.y() = p.y();
	pos.z() = p.z();
	ori.w() = o.w();
	ori.x() = o.x();
	ori.y() = o.y();
	ori.z() = o.z();
}


Point3 transform(const Point3 &psrc, tf::StampedTransform &tfsource)
{
	tf::Vector3 pt3(psrc.x(), psrc.y(), psrc.z());
	tf::Vector3 pt3s = tfsource * pt3;
	return Point3(pt3s.x(), pt3s.y(), pt3s.z());
}


/*
 * Project a point from world coordinate to image plane
 */
bool project2(const Point3 &pt, int &u, int &v, bool useOpenGLCoord = false)
{
	float nearPlane = 1.0;
	float farPlane = 200.0;
	Point3 _pt = transform(pt, trf);
	float _u = _pt.x() * fx / _pt.z() + cx;
	float _v = _pt.y() * fy / _pt.z() + cy;

	u = static_cast<int>(_u);
	v = static_cast<int>(_v);
        std::cout << "u : " << u << " : " << imageWidth << std::endl;
        std::cout << "u : " << v << " : " << imageHeight << std::endl;
        std::cout << "u : " << _pt.z() << " : " << nearPlane << std::endl;
        std::cout << "u : " << _pt.z() << " : " << farPlane << std::endl;
	if (u < 0 || imageWidth < u || v < 0 || imageHeight < v || _pt.z() < nearPlane || farPlane < _pt.z())
	{
		u = -1, v = -1;
		return false;
	}

	if (useOpenGLCoord)
	{
		v = imageHeight - v;
	}

	return true;
}

double ConvertDegreeToRadian(double degree)
{
	return degree * M_PI / 180.0f;
}


double ConvertRadianToDegree(double radian)
{
	return radian * 180.0f / M_PI;
}


double GetSignalAngleInCameraSystem(double hang, double vang)
{
	// Fit the vector map format into ROS style
	double signal_pitch_in_map = ConvertDegreeToRadian(vang - 90);
	double signal_yaw_in_map = ConvertDegreeToRadian(-hang + 90);

	tf::Quaternion signal_orientation_in_map_system;
	signal_orientation_in_map_system.setRPY(0, signal_pitch_in_map, signal_yaw_in_map);

	tf::Quaternion signal_orientation_in_cam_system = trf * signal_orientation_in_map_system;
	double signal_roll_in_cam;
	double signal_pitch_in_cam;
	double signal_yaw_in_cam;
	tf::Matrix3x3(signal_orientation_in_cam_system).getRPY(signal_roll_in_cam,
	                                                       signal_pitch_in_cam,
	                                                       signal_yaw_in_cam);

	return ConvertRadianToDegree(signal_pitch_in_cam);   // holizontal angle of camera is represented by pitch
}  // double GetSignalAngleInCameraSystem()


void echoSignals2(ros::Publisher &pub, bool useOpenGLCoord = false)
{
        int countPoint = 0;
	autoware_msgs::Signals signalsInFrame;


	{
                std::vector<autoware_msgs::ExtractedPosition> ep = roi_signal.Signals;
                for(int i=0; i<ep.size(); i++)
                {
                    autoware_msgs::ExtractedPosition center_ep = ep[i];

                    Point3 signalcenter;
                    signalcenter.x() = center_ep.x;  signalcenter.y() = center_ep.y;  signalcenter.z() = center_ep.z;
                    Point3 signalcenterx(signalcenter.x(), signalcenter.y(), signalcenter.z() + SignalLampRadius);
    std::cout << "signal center : " << signalcenter << std::endl;
                    int u, v;
                    if (project2(signalcenter, u, v, useOpenGLCoord) == true)
                    {
                            countPoint++;
                            // std::cout << u << ", " << v << ", " << std::endl;

                            int radius;
                            int ux, vx;
                            project2(signalcenterx, ux, vx, useOpenGLCoord);
                            radius = (int) distance(ux, vx, u, v);

                            autoware_msgs::ExtractedPosition sign;
                            sign.signalId = center_ep.signalId;

                            sign.u = u + adjust_proj_x; // shift project position by configuration value from runtime manager
                            sign.v = v + adjust_proj_y; // shift project position by configuration value from runtime manager

                            sign.radius = radius;
                            sign.x = signalcenter.x(), sign.y = signalcenter.y(), sign.z = signalcenter.z();
                            sign.hang = center_ep.hang; // hang is expressed in [0, 360] degree
                            sign.type = center_ep.type, sign.linkId = center_ep.linkId;
                            sign.plId = center_ep.plId;

                            // Get holizontal angle of signal in camera corrdinate system
                            double signal_angle = GetSignalAngleInCameraSystem(center_ep.hang + 180.0f,
                                                                               90.0 + 180.0f);

                            // signal_angle will be zero if signal faces to x-axis
                            // Target signal should be face to -50 <= z-axis (= 90 degree) <= +50
                            std::cout << signal_angle << std::endl;
                            if (isRange(-50, 50, signal_angle - 90))
                            {
                                    signalsInFrame.Signals.push_back(sign);
                            }
                    }
                }
	}
	signalsInFrame.header.stamp = ros::Time::now();
	pub.publish(signalsInFrame);

	std::cout << "There are " << signalsInFrame.Signals.size() << " signals in range" << std::endl;
}


void interrupt(int s)
{
	ros::shutdown();
	exit(1);
}


int main(int argc, char *argv[])
{
        ros::init(argc, argv, "feat_proj_loaderGet", ros::init_options::NoSigintHandler);
	ros::NodeHandle rosnode;
	ros::NodeHandle private_nh("~");
	std::string cameraInfo_topic_name;
	private_nh.param<std::string>("camera_info_topic", cameraInfo_topic_name, "/camera_info");

	/* get camera ID */
	camera_id_str = cameraInfo_topic_name;
	camera_id_str.erase(camera_id_str.find("/camera_info"));
	if (camera_id_str == "/")
	{
		camera_id_str = "camera";
	}

	ros::Subscriber cameraInfoSubscriber = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
	ros::Subscriber cameraImage = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
	ros::Subscriber adjust_xySubscriber = rosnode.subscribe("/config/adjust_xy", 100, adjust_xyCallback);
        ros::Subscriber loader_roi_signal_subscriber = rosnode.subscribe("/loader_roi_signal", 100, loader_roi_signal_Callback);
	ros::Publisher signalPublisher = rosnode.advertise<autoware_msgs::Signals>("roi_signal", 100);
	signal(SIGINT, interrupt);

	Rate loop(50);
	Eigen::Vector3f prev_position(0,0,0);
	Eigen::Quaternionf prev_orientation(0,0,0,0);
	while (true)
	{
		ros::spinOnce();

		try
		{
			getTransform(orientation, position);
		}
		catch (tf::TransformException &exc)
		{
		}

		if (prev_orientation.vec() != orientation.vec()  &&
		    prev_position != position)
		{
			echoSignals2(signalPublisher, false);
		}
		prev_orientation = orientation;
		prev_position = position;
		loop.sleep();
	}

}
