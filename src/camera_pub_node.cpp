#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

template<class T>
inline void get_param(const ros::NodeHandle& nh, const std::string& param_name, T& var, const T default_value)
{
    nh.param(param_name, var, default_value);
    ROS_INFO_STREAM("Param " << param_name << ": " << var);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_pub_node");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    std::string videoPath;
    get_param<std::string>(n_priv, "video_path", videoPath, "/home/lingkang/Videos/bebop/Test_waving.mp4");
    bool compressImage;
    get_param<bool>(n_priv, "compress_image", compressImage, true);
    bool debugImage;
    get_param<bool>(n_priv, "debug_image", debugImage, true);
    int frameRate;
    get_param<int>(n_priv, "frame_rate", frameRate, 20);

    cv::VideoCapture cap(videoPath);
    
    if(!cap.isOpened())
    {
        std::cerr << "Unable to load video" << std::endl;
        return 1;
    }

    image_transport::ImageTransport it(n);
    image_transport::Publisher camPub = it.advertise("/usb_cam_node/image_raw", 1);

    ros::Rate loop_rate(frameRate);
    
    cv::namedWindow("video", 1);

    if(!debugImage)
    {
        cv::destroyWindow("video"); 
    }

    while(ros::ok())
    {
        cv::Mat temp0;
        cap >> temp0;
        if(temp0.empty())
        {
            ROS_INFO("The end of the video.");
            break;
        }
        if(debugImage)
        {
            cv::imshow("video", temp0);
            cv::waitKey(1);
        }
        cv::Mat temp;
        if(compressImage)
        {
            cv::resize(temp0, temp, cv::Size(), 0.5, 0.5);
        }
        cv_bridge::CvImage cvi;
        sensor_msgs::Image im;
        cvi.header.frame_id = "image";
        cvi.header.stamp = ros::Time::now();
        cvi.encoding = "bgr8";
        if(compressImage)
        {
            cvi.image = temp;
        }
        else
        {
            cvi.image=temp0;
        }
        cvi.toImageMsg(im);
        camPub.publish(im);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
