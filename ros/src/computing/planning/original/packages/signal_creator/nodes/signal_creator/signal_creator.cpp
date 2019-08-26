#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>

enum class ProcessingID : int
{
    UNKNOWN           = 0,
    ARROW_UP          = 1,
    ARROW_DOWN        = 2,
    ARROW_RIGHT       = 3,
    ARROW_LEFT        = 4,
    PAGE_UP           = 5,
    PAGE_DOWN         = 6,
    CTRL_ARROW_UP     = 7,
    CTRL_ARROW_DOWN   = 8,
    CTRL_ARROW_RIGHT  = 9,
    CTRL_ARROW_LEFT   = 10,
    CTRL_PAGE_UP      = 11,
    CTRL_PAGE_DOWN    = 12,
    SMALL_S           = 13,
};

class ProcessingKey
{
public:
    std::vector<char> key;
    ProcessingID Processing_id;
};

class TF_COORDINATE
{
public:
    tf::Transform coord;
    std::string frame_id;
    int signal_type;
};

class SignalCreator
{
private:
    tf::TransformBroadcaster *broadcaster;
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber sub_clicked_point_;
    std::string clicked_point_topic_;

    std::vector<TF_COORDINATE> tf_coordinate_; //frist data is center
    bool clicked_point_flag_;
    double move_value, rotation_value;
    ros::Time publish_time_;

    void callback_clicked_point(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        std::cout << "\npublished " << clicked_point_topic_ << std::endl;
        std::cout << "x : " << msg->point.x << std::endl;
        std::cout << "y : " << msg->point.y << std::endl;
        std::cout << "z : " << msg->point.z << std::endl;
        std::cout << "--------" << std::endl;

        tf::Transform yellow;
        yellow.setOrigin(tf::Vector3(msg->point.x, msg->point.y, msg->point.z));
        yellow.setRotation(tf::Quaternion(0, 0, 0));
        TF_COORDINATE tc_yellow;
        tc_yellow.coord = yellow;  tc_yellow.frame_id = "/signal_yellow";
        tc_yellow.signal_type = 3;
        tf_coordinate_.push_back(tc_yellow);

        tf::Transform blue;
        blue.setOrigin(tf::Vector3(0.3, 0, 0));
        blue.setRotation(tf::Quaternion(0, 0, 0));
        TF_COORDINATE tc_blue;
        tc_blue.coord = blue;  tc_blue.frame_id = "/signal_blue";
        tc_blue.signal_type = 1;
        tf_coordinate_.push_back(tc_blue);

        tf::Transform red;
        red.setOrigin(tf::Vector3(-0.3, 0, 0));
        red.setRotation(tf::Quaternion(0, 0, 0));
        TF_COORDINATE tc_red;
        tc_red.coord = red;  tc_red.frame_id = "/signal_red";
        tc_red.signal_type = 2;
        tf_coordinate_.push_back(tc_red);

        clicked_point_flag_ = true;
    }
public:
    SignalCreator(ros::NodeHandle nh, ros::NodeHandle p_nh)
        : clicked_point_topic_("/clicked_point")
        , move_value(0.1)
        , rotation_value(0.2*M_PI/180)
    {
        nh_ = nh;  private_nh_ = p_nh;

        broadcaster = new tf::TransformBroadcaster();
        clicked_point_flag_ = false;

        sub_clicked_point_ = nh_.subscribe<geometry_msgs::PointStamped>(clicked_point_topic_, 1,
                                 &SignalCreator::callback_clicked_point, this);
    }

    ~SignalCreator()
    {
        delete broadcaster;
    }

    bool publish_tf()
    {
        if(clicked_point_flag_)
        {
            publish_time_ = ros::Time::now();
            TF_COORDINATE center = tf_coordinate_[0];
            std::string parent_frame = center.frame_id;
            broadcaster->sendTransform(tf::StampedTransform(
                                           center.coord, publish_time_, "/map", parent_frame));
            for(int i=1; i<tf_coordinate_.size(); i++)
            {
                TF_COORDINATE tc = tf_coordinate_[i];
                broadcaster->sendTransform(tf::StampedTransform(
                                               tc.coord, publish_time_, parent_frame, tc.frame_id));
            }
            //broadcaster->sendTransform(tf::StampedTransform(tf_coordinate_, publish_time_,
            //                                               "/map", "/signal_creator"));
            return true;
        }
        return false;
    }

    void printf_tf()
    {
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        tf::Quaternion rotation = tf_coordinate_[0].coord.getRotation();
        std::cout << "origin   : " << origin.x() << "," << origin.y() << "," << origin.z() << std::endl;
        std::cout << "rotation : " << rotation.x() << "," << rotation.y() << "," << rotation.z() << "," << rotation.w() << std::endl;
        tf::Matrix3x3 qua_mat(rotation);
        double yaw, roll, pitch;
        qua_mat.getRPY(roll, pitch, yaw);
        std::cout << "RPY : " << roll << "," << pitch << "," << yaw << std::endl;
        publish_tf();
    }

    void tf_x_up()
    {
        if(clicked_point_flag_ == false) return;
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        origin.setX(origin.getX() + move_value);
        tf_coordinate_[0].coord.setOrigin(origin);
        publish_tf();
    }

    void tf_x_down()
    {
        if(clicked_point_flag_ == false) return;
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        origin.setX(origin.getX() - move_value);
        tf_coordinate_[0].coord.setOrigin(origin);
        publish_tf();
    }

    void tf_y_up()
    {
        if(clicked_point_flag_ == false) return;
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        origin.setY(origin.getY() + move_value);
        tf_coordinate_[0].coord.setOrigin(origin);
        publish_tf();
    }

    void tf_y_down()
    {
        if(clicked_point_flag_ == false) return;
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        origin.setY(origin.getY() - move_value);
        tf_coordinate_[0].coord.setOrigin(origin);
        publish_tf();
    }

    void tf_z_up()
    {
        if(clicked_point_flag_ == false) return;
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        origin.setZ(origin.getZ() + move_value);
        tf_coordinate_[0].coord.setOrigin(origin);
        publish_tf();
    }

    void tf_z_down()
    {
        if(clicked_point_flag_ == false) return;
        tf::Vector3 origin = tf_coordinate_[0].coord.getOrigin();
        origin.setZ(origin.getZ() - move_value);
        tf_coordinate_[0].coord.setOrigin(origin);
        publish_tf();
    }

    void tf_rotx_up()
    {
        if(clicked_point_flag_ == false) return;
        tf::Quaternion rot1 = tf_coordinate_[0].coord.getRotation();
        tf::Quaternion rot2(tf::Vector3(1,0,0), rotation_value);
        tf_coordinate_[0].coord.setRotation(rot1*rot2);
        publish_tf();
    }

    void tf_rotx_down()
    {
        if(clicked_point_flag_ == false) return;
        tf::Quaternion rot1 = tf_coordinate_[0].coord.getRotation();
        tf::Quaternion rot2(tf::Vector3(1,0,0), -rotation_value);
        tf_coordinate_[0].coord.setRotation(rot1*rot2);
        publish_tf();
    }

    void tf_roty_up()
    {
        if(clicked_point_flag_ == false) return;
        tf::Quaternion rot1 = tf_coordinate_[0].coord.getRotation();
        tf::Quaternion rot2(tf::Vector3(0,1,0), rotation_value);
        tf_coordinate_[0].coord.setRotation(rot1*rot2);
        publish_tf();
    }

    void tf_roty_down()
    {
        if(clicked_point_flag_ == false) return;
        tf::Quaternion rot1 = tf_coordinate_[0].coord.getRotation();
        tf::Quaternion rot2(tf::Vector3(0,1,0), -rotation_value);
        tf_coordinate_[0].coord.setRotation(rot1*rot2);
        publish_tf();
    }

    void tf_rotz_up()
    {
        if(clicked_point_flag_ == false) return;
        tf::Quaternion rot1 = tf_coordinate_[0].coord.getRotation();
        tf::Quaternion rot2(tf::Vector3(0,0,1), rotation_value);
        tf_coordinate_[0].coord.setRotation(rot1*rot2);
        publish_tf();
    }

    void tf_rotz_down()
    {
        if(clicked_point_flag_ == false) return;
        tf::Quaternion rot1 = tf_coordinate_[0].coord.getRotation();
        tf::Quaternion rot2(tf::Vector3(0,0,1), -rotation_value);
        tf_coordinate_[0].coord.setRotation(rot1*rot2);
        publish_tf();
    }

    void save(std::string directory)
    {
		std::string signal_path = "signaldata.csv";
		std::string vector_path = "vector.csv";
		std::string point_path  = "point.csv";
		//std::ofstream ofs_signal, ofs_vector, ofs_point;
		//ofs_signal.clear(); ofs_vector.clear(); ofs_point.clear();

		try
		{
			/*ofs_signal.exceptions(std::ios::badbit | std::ios::eofbit | std::ios::failbit);
			ofs_vector.exceptions(std::ios::badbit | std::ios::eofbit | std::ios::failbit);
			ofs_point.exceptions(std::ios::badbit | std::ios::eofbit | std::ios::failbit);
			ofs_signal.open(signal_path.c_str());
			ofs_vector.open(vector_path.c_str());
			ofs_point.open(point_path.c_str());*/

			/*ofs_signal << "ID,VID,PLID,Type,LinkID";
			ofs_vector << "VID,PID,Hang,Vang";
			ofs_point  << "PID,B,L,H,Bx,Ly,Ref,MCODE1,MCODE2,MCODE3";*/
			std::cout << "ID,VID,PLID,Type,LinkID" << std::endl;
			std::cout << "VID,PID,Hang,Vang" << std::endl;
			std::cout << "PID,B,L,H,Bx,Ly,Ref,MCODE1,MCODE2,MCODE3" << std::endl;

			tf::Transform parent_tf;
			for(int cou=0; cou<tf_coordinate_.size(); cou++)
			{
				TF_COORDINATE tf_coord = tf_coordinate_[cou];

				//signal
				/*ofs_signal << "\n" << cou+1; //ID
				ofs_signal << "," << cou+1; //VID
				ofs_signal << ",0"; //PLID
				ofs_signal << "," << tf_coord.signal_type; //Type
				ofs_signal << ",0"; //LinkID*/
				std::cout << "signal output" << std::endl;
				std::cout << cou+1 << "," << cou+1 << ",0," << tf_coord.signal_type << ",0" << std::endl;

				//vector
				double yaw, roll, pitch;
				tf::Vector3 origin;
				if(cou == 0)
				{
					parent_tf = tf_coord.coord;
					origin = tf_coord.coord.getOrigin();
					tf::Quaternion qua = tf_coord.coord.getRotation();
					tf::Matrix3x3 qua_mat(qua);
					qua_mat.getRPY(roll, pitch, yaw);
					/*ofs_vector << "\n" << cou+1; //VID
					ofs_vector << "," << cou+1; //PID
					ofs_vector << "," << (-yaw)*180/M_PI; //Hang
					ofs_vector << "," << roll*180/M_PI+90; //Vang*/
					std::cout << "vector output" << std::endl;
					std::cout << cou+1 << "," << cou+1 << "," << (-yaw)*180/M_PI << "," << roll*180/M_PI+90 << std::endl;
				}
				else
				{
					try
					{
						tf::Transform pose = parent_tf * tf_coord.coord;
						origin = pose.getOrigin();
						tf::Quaternion qua = pose.getRotation();
						tf::Matrix3x3 qua_mat(qua);
						qua_mat.getRPY(roll, pitch, yaw);
						/*ofs_vector << "\n" << cou+1; //VID
						ofs_vector << "," << cou+1; //PID
						ofs_vector << "," << std::setprecision(10) << (-yaw)*180/M_PI; //Hang
						ofs_vector << "," << std::setprecision(10) << roll*180/M_PI+90; //Vang*/
						std::cout << "vector output" << std::endl;
						std::cout << cou+1 << "," << cou+1 << "," << (-yaw)*180/M_PI << "," << roll*180/M_PI+90 << std::endl;
					}
					catch(tf::TransformException ex){
						std::cerr << "transform error" << std::endl;
					}
				}


				//point
				/*ofs_point << "\n" << cou+1; //PID
				ofs_point << ",0"; // B
				ofs_point << ",0"; // L
				ofs_point << "," << std::setprecision(10) << origin.getZ(); //H
				ofs_point << "," << std::setprecision(10) << origin.getY(); //Bx
				ofs_point << "," << std::setprecision(10) << origin.getX(); //Ly
				ofs_point << ",0,0,0,0";*/
				std::cout << "point output" << std::endl;
				std::cout << cou+1 << ",0,0," << std::setprecision(10) << origin.getZ() << "," << std::setprecision(10) << origin.getY() << "," << std::setprecision(10) << origin.getX() << ",0,0,0,0" << std::endl;
			}

			//ofs_signal.close();  ofs_vector.close();  ofs_vector.close();

		}
		catch(std::exception &e)
		{
			std::cerr << "error : " << e.what() << " : " << errno << std::endl;
		}
    }
};

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void print_description()
{
    std::cout << "Move with arrow key.\n";
    std::cout << "Rotate with arrow key + shift key.\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "signal_creator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string save_directory;
    private_nh.param<std::string>("save_directory", save_directory, ".");
	std::cout << "save directory : " << save_directory << std::endl;
    print_description();

    SignalCreator signal_creator(nh, private_nh);


    ProcessingKey arrow_up = {{27,91,65}, ProcessingID::ARROW_UP};
    ProcessingKey arrow_down = {{27,91,66}, ProcessingID::ARROW_DOWN};
    ProcessingKey arrow_right= {{27,91,67}, ProcessingID::ARROW_RIGHT};
    ProcessingKey arrow_left= {{27,91,68}, ProcessingID::ARROW_LEFT};
    ProcessingKey page_up = {{27,91,53,126}, ProcessingID::PAGE_UP};
    ProcessingKey page_down = {{27,91,54,126}, ProcessingID::PAGE_DOWN};
    ProcessingKey ctrl_arrow_up= {{27,91,49,59,53,65}, ProcessingID::CTRL_ARROW_UP};
    ProcessingKey ctrl_arrow_down= {{27,91,49,59,53,66}, ProcessingID::CTRL_ARROW_DOWN};
    ProcessingKey ctrl_arrow_right= {{27,91,49,59,53,67}, ProcessingID::CTRL_ARROW_RIGHT};
    ProcessingKey ctrl_arrow_left= {{27,91,49,59,53,68}, ProcessingID::CTRL_ARROW_LEFT};
    ProcessingKey ctrl_page_up= {{27,91,53,59,53,126}, ProcessingID::CTRL_PAGE_UP};
    ProcessingKey ctrl_page_down= {{27,91,54,59,53,126}, ProcessingID::CTRL_PAGE_DOWN};
    ProcessingKey small_s ={{115}, ProcessingID::SMALL_S};
    std::vector<ProcessingKey> processing_keys =
        {
            arrow_up, arrow_down, arrow_right, arrow_left, page_up, page_down,
            ctrl_arrow_up, ctrl_arrow_down, ctrl_arrow_right, ctrl_arrow_left, ctrl_page_up, ctrl_page_down,
            small_s
        };

    std::vector<char> key_cache;
    while(ros::ok())
    {
        tf::Transform tf_coordinate;
        bool tf_publish_flag = signal_creator.publish_tf();

        if (kbhit())
        {
            int key = getchar();
            std::cout << "key : " << key << std::endl;
            key_cache.push_back(key);
        }
        else if(key_cache.size() > 0)
        {
            for(ProcessingKey pro_key : processing_keys)
            {
                if(key_cache == pro_key.key)
                {
                    switch(pro_key.Processing_id)
                    {
                    case ProcessingID::ARROW_UP:
                        {
                            std::cout << "move up" << std::endl;
                            signal_creator.tf_y_up();
                            break;
                        }
                    case ProcessingID::ARROW_DOWN:
                        {
                            std::cout << "move down" << std::endl;
                            signal_creator.tf_y_down();
                            break;
                        }
                    case ProcessingID::ARROW_RIGHT:
                        {
                            std::cout << "move right" << std::endl;
                            signal_creator.tf_x_up();
                            break;
                        }
                    case ProcessingID::ARROW_LEFT:
                        {
                            std::cout << "move left" << std::endl;
                            signal_creator.tf_x_down();
                            break;
                        }
                    case ProcessingID::PAGE_UP:
                        {
                            std::cout << "move previous" << std::endl;
                            signal_creator.tf_z_up();
                            break;
                        }
                    case ProcessingID::PAGE_DOWN:
                        {
                            std::cout << "move back" << std::endl;
                            signal_creator.tf_z_down();
                            break;
                        }
                    case ProcessingID::CTRL_ARROW_UP:
                        {
                            std::cout << "rotation up" << std::endl;
                            signal_creator.tf_rotx_up();
                            break;
                        }
                    case ProcessingID::CTRL_ARROW_DOWN:
                        {
                            std::cout << "rotation_down" << std::endl;
                            signal_creator.tf_rotx_down();
                            break;
                        }
                    case ProcessingID::CTRL_ARROW_RIGHT:
                        {
                            std::cout << "rotation_right" << std::endl;
                            signal_creator.tf_rotz_down();
                            break;
                        }
                    case ProcessingID::CTRL_ARROW_LEFT:
                        {
                            std::cout << "rotation_left" << std::endl;
                            signal_creator.tf_rotz_up();
                            break;
                        }
                    case ProcessingID::CTRL_PAGE_UP:
                        {
                            std::cout << "rotation_previous" << std::endl;
                            signal_creator.tf_roty_down();
                            break;
                        }
                    case ProcessingID::CTRL_PAGE_DOWN:
                        {
                            std::cout << "rotation_back" << std::endl;
                            signal_creator.tf_roty_up();
                            break;
                        }
                    case ProcessingID::SMALL_S:
                        {
                            std::cout << "files save" << std::endl;
                            signal_creator.save(save_directory);
                        }
                    default:
                        {
                            break;
                        }
                    }

                    signal_creator.printf_tf();
                }
            }

            std::cout << "--------" << std::endl;
            key_cache.clear();
        }
        ros::spinOnce();
    }
    return 0;
}
