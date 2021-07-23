#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

#include <zed-open-capture/sensorcapture.hpp>
#include <zed-open-capture/videocapture.hpp>

void sensor_callback(const ros::TimerEvent &timer_event)
{
    const sl_oc::sensors::data::Imu imuData = zed->getLastIMUData();
    if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) // Uncomment to use only data syncronized with the video frames
    {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time(imuData.timestamp * 1e-9);
        imu.header.frame_id = "world";
        imu.angular_velocity.x = imuData.gX;
        imu.angular_velocity.y = imuData.gY;
        imu.angular_velocity.z = imuData.gZ;
        imu.linear_acceleration.x = imuData.aX;
        imu.linear_acceleration.y = imuData.aY;
        imu.linear_acceleration.z = imuData.aZ;
        sensor_pub.publish(imu);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // setup publisher stuff
    image_transport::ImageTransport it(nh);
    left_image_pub = it.advertise("left/image_raw", 30);
    right_image_pub = it.advertise("right/image_raw", 30);
    sensor_pub = nh.advertise<sensor_msgs::Imu>("imu/raw", 200);

    ros::Timer image_timer;
    ros::Timer sensor_timer;

    // get ros param
    int resolution;
    private_nh.param("resolution", resolution, 3);
    resolution=3;
    std::cout<<"Resulution: "<<resolution<<std::endl;
    correctFramerate(resolution);
    zed = new StereoCamera(gResolution, gFps);
    if (zed->camera)
    {
        image_timer = nh.createTimer(ros::Duration(0.01), image_callback);
    }
    if (zed->sensor)
    {
        sensor_timer = nh.createTimer(ros::Duration(0.001), sensor_callback);
    }
    if (!zed->camera && !zed->sensor)
    {
        ROS_ERROR("Fail to Initialize the camera");
        return EXIT_FAILURE;
    }
    ros::spin();
}
