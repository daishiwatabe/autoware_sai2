#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <autoware_config_msgs/ConfigCameraROI.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/Format7Info.h>
#include <FlyCapture2.h>
#include <limits.h>

class ROI
{
public:
    int left, top, width, height;
};

class PrivateNodeParams
{
public:
    int fps_, timeout_;
    int serial_number_;
    FlyCapture2::Mode format7mode_;
    FlyCapture2::PixelFormat pixel_format_;
    bool size_extension_flag_; //現在のモードでの最大画像で出力するかのフラグ
    ROI roi_;
    int signal_roi_extension;
};

class Camera
{
private:
    ros::NodeHandle nh_, private_nh_;
    FlyCapture2::Camera *camera_;
    PrivateNodeParams private_params_;
    int capture_counter_; //画像キャプチャー回数
    std::string frame_id_;

    ros::Subscriber sub_roi_change_; //カメラROIチェンジ指令
    ros::Subscriber sub_roi_Signals_; //信号機情報に合わせてROIを変更
    ros::Publisher  pub_format7info_; //現在のモードのforfmat7の基本情報をpublish
    ros::Publisher  pub_capture_image_; //キャプチャーした画像をpublish

    //ノード起動時用パラメータを取得
    bool ros_get_params(PrivateNodeParams &params)
    {
            if (private_nh_.getParam("serial", params.serial_number_))
            {
                    ROS_INFO("serial_number set to %d", params.serial_number_);
            } else {
                    ROS_INFO("No param received, serial_number nothing");
                    return false;
            }

            if (private_nh_.getParam("fps", params.fps_))
            {
                    ROS_INFO("fps set to %d", params.fps_);
            } else {
                    params.fps_ = 20;
                    ROS_INFO("No param received, defaulting fps to %d", params.fps_);
            }

            int mode;
            if (private_nh_.getParam("mode", mode))
            {
                    params.format7mode_ = (FlyCapture2::Mode)mode;
                    ROS_INFO("mode set to %d", mode);
            } else {
                    params.format7mode_ = (FlyCapture2::Mode)0;
                    ROS_INFO("No param received, defaulting mode to %d", mode);
            }

            std::string format;
            if (private_nh_.getParam("format", format))
            {
                    params.pixel_format_ = (format == "raw") ? FlyCapture2::PIXEL_FORMAT_RAW8 : FlyCapture2::PIXEL_FORMAT_RGB8;
                    ROS_INFO("format set to %s", format.c_str());
            } else {
                    params.pixel_format_ = FlyCapture2::PIXEL_FORMAT_RAW8;
                    ROS_INFO("No param received, defaulting format to %s", format.c_str());
            }

            if (private_nh_.getParam("timeout", params.timeout_))
            {
                    ROS_INFO("timeout set to %d ms", params.timeout_);
            } else {
                    params.timeout_ = 1000;
                    ROS_INFO("No param received, defaulting timeout to %d ms", params.timeout_);
            }

            if (private_nh_.getParam("size_extension_flag", params.size_extension_flag_))
            {
                    ROS_INFO("size_extension_flag set to %s", (params.size_extension_flag_ ? "true" : "false"));
            } else {
                    params.size_extension_flag_ = true;
                    ROS_INFO("No param received, defaulting timeout to %s", (params.size_extension_flag_ ? "true" : "false"));
            }

            if (private_nh_.getParam("roi_left", params.roi_.left))
            {
                    ROS_INFO("roi_left set to %d", params.roi_.left);
            } else {
                    params.roi_.left = 0;
                    ROS_INFO("No param received, defaulting roi_left to %d", params.roi_.left);
            }

            if (private_nh_.getParam("roi_top", params.roi_.top))
            {
                    ROS_INFO("roi_top set to %d", params.roi_.top);
            } else {
                    params.roi_.top = 0;
                    ROS_INFO("No param received, defaulting roi_top to %d", params.roi_.top);
            }

            if (private_nh_.getParam("roi_width", params.roi_.width))
            {
                    ROS_INFO("roi_width set to %d", params.roi_.width);
            } else {
                    params.roi_.width = 0;
                    ROS_INFO("No param received, defaulting roi_width to %d", params.roi_.width);
            }

            if (private_nh_.getParam("roi_height", params.roi_.height))
            {
                    ROS_INFO("roi_height set to %d", params.roi_.height);
            } else {
                    params.roi_.height = 0;
                    ROS_INFO("No param received, defaulting roi_height to %d", params.roi_.height);
            }

            if (private_nh_.getParam("signal_roi_extension", params.signal_roi_extension))
            {
                    ROS_INFO("signal_roi_extension set to %d", params.signal_roi_extension);
            } else {
                    params.signal_roi_extension = 30;
                    ROS_INFO("No param received, defaulting signal_roi_extension to %d", params.signal_roi_extension);
            }

            return true;
    }

    FlyCapture2::Camera* getCamera(FlyCapture2::BusManager &manager, int serial)
    {
        FlyCapture2::PGRGuid guid;
        FlyCapture2::Error error = manager.GetCameraFromSerialNumber(serial, &guid);
        if(error != FlyCapture2::PGRERROR_OK) {error.PrintErrorTrace(); return NULL;}

        FlyCapture2::Camera *camera = new FlyCapture2::Camera();
        error = camera->Connect(&guid);
        if(error != FlyCapture2::PGRERROR_OK) {error.PrintErrorTrace(); return NULL;}

        return camera;
    }

    bool set_embedded_image_info()
    {
        FlyCapture2::EmbeddedImageInfo image_info;
        FlyCapture2::Error error = camera_->GetEmbeddedImageInfo(&image_info);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        image_info.timestamp.onOff = false;
        error = camera_->SetEmbeddedImageInfo(&image_info);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        return true;
    }

    int get_format7Info(FlyCapture2::Format7Info &format7_info, FlyCapture2::Mode format7mode)
    {
        bool supported;
        format7_info.mode = format7mode;

        FlyCapture2::Error error = camera_->GetFormat7Info(&format7_info, &supported);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return -1;
        }

        return (supported == false) ? 0 : 1;
    }

    bool get_format7_configuration(FlyCapture2::Format7ImageSettings &image_settings,
                                   unsigned int &packet_size,
                                   float &percentage)
    {
        FlyCapture2::Error error = camera_->GetFormat7Configuration(
                    &image_settings, &packet_size, &percentage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        return true;
    }

    bool set_format7_configuration(const FlyCapture2::Format7Info &format7info,
                                   const FlyCapture2::PixelFormat &format7pixel_format,
                                   const ROI &roi)
    {
        FlyCapture2::Format7ImageSettings image_settings;
        unsigned int packet_size;
        float percentage;
        if(false == get_format7_configuration(image_settings, packet_size, percentage))
            return false;
        /*FlyCapture2::Error error = camera_->GetFormat7Configuration(
                    &image_settings, &packet_size, &percentage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }*/

        unsigned int h_step = format7info.imageHStepSize;
        unsigned int v_step = format7info.imageVStepSize;
        std::cout << "Hstep : " << h_step << std::endl;
        std::cout << "Vstep : " << v_step << std::endl;

        image_settings.mode = format7info.mode;
        image_settings.pixelFormat = format7pixel_format;
        ROI roicp = roi;
        if(roicp.left < 0) roicp.left = 0;
        if(roicp.top < 0) roicp.top = 0;
        if(roicp.width > format7info.maxWidth) roicp.width = format7info.maxWidth;
        if(roicp.height > format7info.maxHeight) roicp.height = format7info.maxHeight;
        image_settings.offsetX = (unsigned int)(std::round((double)roicp.left/h_step))*h_step;
        image_settings.offsetY = (unsigned int)(std::round((double)roicp.top/v_step))*v_step;
        image_settings.width = (unsigned int)(std::round((double)roicp.width/h_step))*h_step;
        image_settings.height = (unsigned int)(std::round((double)roicp.height/v_step))*v_step;

        //image_settings.offsetX = (roi.left <= 0) ? 0 : (roi.left/h_step)*h_step;
        //image_settings.offsetY = (roi.top <= 0) ? 0 : (roi.top/v_step)*v_step;
        //image_settings.offsetX = (roi.width <= 0) ? format7info.maxWidth : (roi.width/h_step)*h_step;
        //image_settings.height = (roi.height <= 0) ? format7info.maxHeight : (roi.height/v_step)*v_step;

        FlyCapture2::Format7PacketInfo packet_info;
        bool valid_settings = false;
        FlyCapture2::Error error = camera_->ValidateFormat7Settings(&image_settings, &valid_settings, &packet_info);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }
        packet_size = packet_info.recommendedBytesPerPacket;
        error = camera_->SetFormat7Configuration(&image_settings, packet_size);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        return true;
    }

    bool print_camera_info()
    {
        // Get the camera information
        FlyCapture2::CameraInfo info;
        FlyCapture2::Error error = camera_->GetCameraInfo(&info);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        std::cout << "\n*** CAMERA INFORMATION ***\n"
                  << "\tSerial number       - " << info.serialNumber << "\n"
                  << "\tCamera model        - " << info.modelName << "\n"
                  << "\tCamera vendor       - " << info.vendorName << "\n"
                  << "\tSendor              - " << info.sensorInfo << "\n"
                  << "\tResolution          - " << info.sensorResolution << "\n"
                  << "\tFirmware version    - " << info.firmwareVersion << "\n"
                  << "\tFirmware build time - " << info.firmwareBuildTime
                  << std::endl;

        FlyCapture2::Format7ImageSettings image_settings;
        unsigned int packet_size;
        float percentage;
        error = camera_->GetFormat7Configuration(&image_settings, &packet_size, &percentage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        std::cout << "Image settings: " << std::endl;
        std::cout << "\tMode: " << image_settings.mode << std::endl;
        std::cout << "\tPixel Format: 0x" << std::hex << image_settings.pixelFormat << std::dec << std::endl;
        std::cout << "\tOffset X: " << image_settings.offsetX << std::endl;
        std::cout << "\tOffset Y: " << image_settings.offsetY << std::endl;
        std::cout << "\tWidth: " << image_settings.width << std::endl;
        std::cout << "\tHeight: " << image_settings.height << std::endl;
        std::cout << "Packet size: " << packet_size << " (" << percentage << "%)" << std::endl;
        return true;
    }

    bool roi_change(const ROI &roi)
    {
        camera_->StopCapture();
        FlyCapture2::Format7Info format7info;
        switch(get_format7Info(format7info, private_params_.format7mode_))
        {
        case -1:
        case 0:
            { return false; }
        }

        set_format7_configuration(format7info, private_params_.pixel_format_, roi);

        camera_->StartCapture();
        return true;
    }

    void callback_roi_change(const autoware_config_msgs::ConfigCameraROI::ConstPtr &msg)
    {
        std::cout << "left : " << msg->left << std::endl;
        std::cout << "top : " << msg->top << std::endl;
        std::cout << "width : " << msg->width << std::endl;
        std::cout << "height : " << msg->height << std::endl;

        ROI roi = {msg->left, msg->top, msg->width, msg->height};
        roi_change(roi);
    }

    void callback_roi_signals(const autoware_msgs::Signals::ConstPtr &msg)
    {
        bool flag = false;
        int u_min = INT_MAX, u_max = INT_MIN, v_min = INT_MAX, v_max = INT_MIN;
        int radius_max = INT_MIN;
        for(autoware_msgs::ExtractedPosition ep : msg->Signals)
        {
            std::cout << "ep.u : " << ep.u << std::endl;
            std::cout << "ep.v : " << ep.v << std::endl;
            std::cout << "radius : " << ep.radius << std::endl;
            if(u_min > ep.u) u_min = ep.u;
            if(u_max < ep.u) u_max = ep.u;
            if(v_min > ep.v) v_min = ep.v;
            if(v_max < ep.v) v_max = ep.v;
            if(radius_max < ep.radius) radius_max = ep.radius;
            flag = true;
        }

        if(flag == true)
        {
            std::cout << "min ep.u : " << u_min << std::endl;
            std::cout << "max ep.u : " << u_max << std::endl;
            std::cout << "min ep.v : " << v_min << std::endl;
            std::cout << "max ep.v : " << v_max << std::endl;
            std::cout << "max radius : " << radius_max << std::endl;
            std::cout << "--------\n";

            ROI roi;
            roi.left   = u_min - radius_max;
            roi.top    = v_min - radius_max;
            roi.width  = u_max + radius_max - roi.left;
            roi.height = v_max + radius_max - roi.top;
            roi.left   -= private_params_.signal_roi_extension;
            roi.top    -= private_params_.signal_roi_extension;
            roi.width  += private_params_.signal_roi_extension*2;
            roi.height += private_params_.signal_roi_extension*2;
            //= {u_min - radius_max, u_max + radius_max,
            //           v_min - radius_max, v_max + radius_max};
            roi_change(roi);
        }
    }
public:
    Camera(ros::NodeHandle nh, ros::NodeHandle p_nh)
        : nh_(nh)
        , private_nh_(p_nh)
        , camera_(NULL)
        , capture_counter_(0)
        , frame_id_("camera")
    {
        //このノード用privateパラメータを取得
        if(!ros_get_params(private_params_)) {throw std::string("node stop");}

        //取得したserial numberのカメラにコネクト
        FlyCapture2::BusManager busMgr;
        camera_ = getCamera(busMgr, private_params_.serial_number_);
        if(camera_ == NULL) {throw std::string("node stop");}
        std::cout << "camera connect OK.\n";

        //flycaputureのcamera settingを設定
        //現在はタイムスタンプを出力しないようにしているだけ
        if(!set_embedded_image_info()) {throw std::string("node stop");}
        std::cout << "camera setting OK.\n";

        //format7infoformat7を取得
        //format7をサポートしているか確認
        FlyCapture2::Format7Info format7info;
        switch(get_format7Info(format7info, private_params_.format7mode_))
        {
        case -1:
            {throw std::string("node stop");}
        case 0:
            {throw std::string("format7 not support");}
        case 1:
            {std::cout << "format7 support OK.\n"; break;}
        }

        //format7設定
        //画像サイズ、ピクセルモード、format7モード、パケット設定はここで行っている
        ROI roi = {private_params_.roi_.left, private_params_.roi_.top,
                   private_params_.roi_.width, private_params_.roi_.height};
        if(!set_format7_configuration(format7info, private_params_.pixel_format_,roi)) //private_params_.roi_))
        {throw std::string("node stop");}

        //camera settingを表示
        if(!print_camera_info()) {throw std::string("node stop");}

        //キャプチャー開始
        FlyCapture2::Error error = camera_->StartCapture();
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            throw std::string("node stop");
        }

        pub_format7info_   = nh_.advertise<autoware_msgs::Format7Info>("/format7info", 10, true);
        autoware_msgs::Format7Info format7info_msg;
        format7info_msg.header.frame_id = frame_id_;
        format7info_msg.header.stamp = ros::Time::now();
        format7info_msg.header.seq = 0;
        format7info_msg.mode = format7info.mode;
        format7info_msg.max_width = format7info.maxWidth;
        format7info_msg.max_height = format7info.maxHeight;
        format7info_msg.offset_hstep_size = format7info.offsetHStepSize;
        format7info_msg.offset_vstep_size = format7info.offsetVStepSize;
        format7info_msg.image_hstep_size = format7info.imageHStepSize;
        format7info_msg.image_vstep_size = format7info.imageVStepSize;
        format7info_msg.pixel_format_bit_field = format7info.pixelFormatBitField;
        format7info_msg.vendor_pixel_format_bit_field = format7info.vendorPixelFormatBitField;
        format7info_msg.packet_size = format7info.packetSize;
        format7info_msg.min_packet_size = format7info.minPacketSize;
        format7info_msg.max_packet_size = format7info.maxPacketSize;
        format7info_msg.percentage = format7info.percentage;
        pub_format7info_.publish(format7info_msg);

        pub_capture_image_ = nh_.advertise<sensor_msgs::Image>("/image_raw", 100);
        sub_roi_change_    = nh_.subscribe<autoware_config_msgs::ConfigCameraROI>("/config/roi_change",
                                                          1, &Camera::callback_roi_change, this);
        sub_roi_Signals_   = nh_.subscribe<autoware_msgs::Signals>("/roi_signal",
                                                          1, &Camera::callback_roi_signals, this);
    }

    ~Camera()
    {
        if(camera_ != NULL)
        {
            camera_->StopCapture();
            camera_->Disconnect();
            delete camera_;
        }
    }

    bool publishImage()
    {
        FlyCapture2::Image image;
        FlyCapture2::Error error = camera_->RetrieveBuffer(&image);
        if (error != FlyCapture2::PGRERROR_OK)
        {
                error.PrintErrorTrace();
                return false;
        }

        // check encoding pattern
        std::string encoding_pattern;
        switch (image.GetBayerTileFormat()) {
        case FlyCapture2::RGGB:
          encoding_pattern = "bayer_rggb8";
          break;
        case FlyCapture2::GRBG:
          encoding_pattern = "bayer_grbg8";
          break;
        case FlyCapture2::GBRG:
          encoding_pattern = "bayer_gbrg8";
          break;
        case FlyCapture2::BGGR:
          encoding_pattern = "bayer_bggr8";
          break;
        default:
          encoding_pattern = "rgb8";
        }

        //publish*******************
        sensor_msgs::Image msg;
        msg.header.seq = capture_counter_;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = ros::Time::now();
        if(private_params_.size_extension_flag_ == false)
        {
            msg.height = image.GetRows();
            msg.width  = image.GetCols();
            msg.encoding = encoding_pattern;
            msg.step = image.GetStride();

            size_t image_size = image.GetDataSize();
            msg.data.resize(image_size);
            memcpy(msg.data.data(), image.GetData(), image_size);

            //std::cout << "height : " << image.GetRows() << std::endl;
            //std::cout << "width : " << image.GetCols() << std::endl;
            //std::cout << "enc : " << encoding_pattern << std::endl;
            //std::cout << "step : " << image.GetStride() << std::endl;
            //std::cout << "image_size : " << image_size << std::endl;
            //std::cout << "aaa : " << image.GetBitsPerPixel() << std::endl;
            pub_capture_image_.publish(msg);
        }
        else
        {
            unsigned int bits_per_pixel = image.GetBitsPerPixel();
            if(bits_per_pixel%8 != 0)//１ピクセルがバイト単位で収まっていない場合は,現状では処理を飛ばします。
            {
                std::cerr << "bits_per_pixel can not be divided by 8.\n";
                return false;
            }
            unsigned int byte_per_pixel = bits_per_pixel / 8;

            FlyCapture2::Format7Info format7info;
            if(!get_format7Info(format7info, private_params_.format7mode_))
                return false;

            FlyCapture2::Format7ImageSettings image_settings;
            unsigned int packet_size;
            float percentage;
            if(!get_format7_configuration(image_settings, packet_size, percentage))
                return false;

            //std::cout << "offsetX : " << image_settings.offsetX << std::endl;
            //std::cout << "offsetX : " << image_settings.offsetY << std::endl;
            msg.height = format7info.maxHeight;
            msg.width  = format7info.maxWidth;
            msg.encoding = encoding_pattern;
            msg.step = msg.width * byte_per_pixel;

            size_t image_size = msg.height * msg.width * byte_per_pixel;
            msg.data.resize(image_size);
            unsigned int offsetX = image_settings.offsetX;
            unsigned int offsetY = image_settings.offsetY;
            unsigned int end_row = offsetY + image.GetRows();
            if(end_row > format7info.maxHeight) end_row = format7info.maxHeight;
            unsigned char *original_data = image.GetData();
            unsigned char *extension_data = msg.data.data();
            unsigned int orig_step = image.GetStride();
            for(unsigned int ext_row=offsetY, orig_row=0; ext_row<end_row; ext_row++, orig_row++)
            {
                unsigned int orig_point = (orig_row * orig_step + 0) * byte_per_pixel;
                unsigned int ext_point = (ext_row * msg.step + offsetX) * byte_per_pixel;
                memcpy(&extension_data[ext_point], &original_data[orig_point], orig_step);
            }

            pub_capture_image_.publish(msg);
        }
        capture_counter_++;
    }

    PrivateNodeParams get_PrivateNodeParams() {return private_params_;}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasshopper3_single");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try
    {
        Camera camera(nh, private_nh);

        std::cout << "camera capture start.\n";
        ros::Rate loop_rate(camera.get_PrivateNodeParams().fps_); // Hz
        //while (running && ros::ok())
        while (ros::ok())
        {
            camera.publishImage();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    catch(std::string error)
    {
        std::cout << error << std::endl;
    }

    return 0;
}
