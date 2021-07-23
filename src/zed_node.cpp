#include <string>
#include <vector>
#include <numeric>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include<Eigen/Core>
#include<Eigen/Dense>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

#include <zed-open-capture/sensorcapture.hpp>
#include <zed-open-capture/videocapture.hpp>

using namespace std;
using namespace message_filters;

class StereoCamera
{
public:
    StereoCamera(sl_oc::video::RESOLUTION resolution, sl_oc::video::FPS fps)
    {
        sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;
        sl_oc::video::VideoParams params;
        params.res = resolution;
        params.fps = fps;
        params.verbose = verbose;

        // Create a Video Capture object
        auto camera_ = new sl_oc::video::VideoCapture(params);
        if (!camera_->initializeVideo(-1))
        {
            std::cerr << "Cannot open camera video capture" << std::endl;
            return;
        }
        else
        {
            camera = camera_;
        }

        // Serial number of the connected camera
        int camSn = camera->getSerialNumber();
        std::cout << "Video Capture connected to camera sn: " << camSn << std::endl;

        // Create a Sensors Capture object
        auto sensor_ = new sl_oc::sensors::SensorCapture(verbose);
        if (!sensor_->initializeSensors(camSn)) // Note: we use the serial number acquired by the VideoCapture object
        {
            std::cerr << "Cannot open sensors capture" << std::endl;
            return;
        }
        else
        {
            sensor = sensor_;
            camera->enableSensorSync(sensor);
        }
    }

    ~StereoCamera()
    {
        delete camera;
        delete sensor;
    }

    // Sensor acquisition runs at 400Hz, so it must be executed in a different thread
    const sl_oc::sensors::data::Imu getLastIMUData()
    {
        return sensor->getLastIMUData();
    }

    const sl_oc::video::Frame getLastFrame()
    {
        return camera->getLastFrame(1);
    }

    sl_oc::video::VideoCapture *camera;
    sl_oc::sensors::SensorCapture *sensor;
};

image_transport::Publisher left_image_pub;
image_transport::Publisher right_image_pub;
ros::Publisher imu_filtered_pub;
// ros::Subscriber odom_sub = n.subscribe("/odometry/filtered", 1000, odomCallback);
// ros::Subscriber acc_sub = n.subscribe("/accel/filtered", 1000, odomCallback);



sensor_msgs::Imu filtered_imu_msg;


ros::Publisher sensor_pub;
sl_oc::video::RESOLUTION gResolution;
sl_oc::video::FPS gFps;
StereoCamera *zed;
Eigen::Vector3d tmp_acc, tmp_gyr, rect_acc, rect_gyr;  

Eigen::Matrix3d acc_miss_align = Eigen::Matrix3d::Identity();

    
Eigen::Matrix3d acc_scale = Eigen::Matrix3d::Identity();

Eigen::Vector3d acc_bias(0.029,0.282,-0.002);

Eigen::Vector3d acc_bias_after_rect(-0.393,0.056,0.00);

const double deg_to_degree_factor=0.0174533;

vector<double> mean_rect_acc2,mean_rect_acc1,mean_rect_acc0;

int cnt=0;

Eigen::Matrix3d gyr_miss_align =Eigen::Matrix3d::Identity();
    
Eigen::Matrix3d gyr_scale = (Eigen::Matrix3d(3,3) << 0.989,  0.0, 0.0, 
                                                                                                                            0.0, 0.974, 0.0,
                                                                                                                            0.0,0.0,1.022).finished();

Eigen::Vector3d gyr_bias(0.0,0.0,-0.004);



Eigen::Matrix3d acc_inv_TK= (acc_miss_align * acc_scale).inverse();
Eigen::Matrix3d acc_TK= acc_miss_align * acc_scale;

Eigen::Matrix3d gyr_inv_TK= (gyr_miss_align *  gyr_scale).inverse();
Eigen::Matrix3d gyr_TK= gyr_miss_align *  gyr_scale;

ros::Time img_ros_time, imu_ros_time;
double img_time,imu_time;

struct imu_rect_param{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    const double inv_scale_acc_x=1.00;
    const double inv_scale_acc_y=1.00;
    const double inv_scale_acc_z=1.00;

    const double bias_acc_x=0.01;
    const double bias_acc_y=-0.29;
    const double bias_acc_z=-0.01;

    const double inv_scale_gyr_x=1.35;
    const double inv_scale_gyr_y=1.33;
    const double inv_scale_gyr_z=1.21;

    const double bias_gyr_x=0.20;
    const double bias_gyr_y=0.07;
    const double bias_gyr_z=0.08;

};



void image_callback(const ros::TimerEvent &timer_event)
{
    static double last_timestamp = 0;
    const sl_oc::video::Frame frame = zed->getLastFrame();
    if (frame.data != nullptr && frame.timestamp != last_timestamp)
    {
        last_timestamp = frame.timestamp;
        cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR(frame.height, frame.width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
        cv::Mat left_image, right_image;
        left_image = frameBGR(cv::Rect(0, 0, frame.width / 2, frame.height));
        right_image = frameBGR(cv::Rect(frame.width / 2, 0, frame.width / 2, frame.height));

        cv_bridge::CvImage cv_left_image;
        cv_left_image.image = left_image;
        cv_left_image.encoding = "bgr8";
        cv_left_image.header.frame_id = "left_frame";
        cv_left_image.header.stamp = ros::Time(last_timestamp * 1e-9);
        img_ros_time=cv_left_image.header.stamp;
        img_time=img_ros_time.toSec();

        cout.setf(ios::fixed,ios::floatfield);
        // cout<< "img time: "<< img_time << endl;
        // cout<<"diff time: "<< fabs(img_time - imu_time )<<endl;

        left_image_pub.publish(cv_left_image.toImageMsg());

        cv_bridge::CvImage cv_right_image;
        cv_right_image.image = right_image;
        cv_right_image.encoding = "bgr8";
        cv_right_image.header.frame_id = "right_frame";
        cv_right_image.header.stamp = ros::Time(last_timestamp * 1e-9);
        right_image_pub.publish(cv_right_image.toImageMsg());
    }
}

void sensor_callback(const ros::TimerEvent &timer_event)
{
    const sl_oc::sensors::data::Imu imuData = zed->getLastIMUData();
    if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) // Uncomment to use only data syncronized with the video frames
    {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time(imuData.timestamp * 1e-9);
        imu_ros_time= imu.header.stamp;
        
        imu_time=imu_ros_time.toSec();

        // cout.setf(ios::fixed,ios::floatfield);
        // cout<< "imu time: "<<  imu_time << endl;

        imu.header.frame_id = "world";
        // imu.angular_velocity.x = imuData.gX;
        // imu.angular_velocity.y = imuData.gY;
        // imu.angular_velocity.z = imuData.gZ;
        tmp_gyr<<-imuData.gZ, -imuData.gY, -imuData.gX;
        tmp_gyr*=deg_to_degree_factor;

        rect_gyr=gyr_TK*(tmp_gyr-gyr_bias);
        // rect_gyr=tmp_gyr; // do not do rectification, use raw measurement

        // imu.angular_velocity.x = rect_gyr[0];
        // imu.angular_velocity.y = rect_gyr[1];
        // imu.angular_velocity.z = rect_gyr[2];

        // imu.angular_velocity.x = -rect_gyr[2];
        // imu.angular_velocity.y = -rect_gyr[1];
        // imu.angular_velocity.z = -rect_gyr[0];

        imu.angular_velocity.x = rect_gyr[0];
        imu.angular_velocity.y = rect_gyr[1];
        imu.angular_velocity.z = rect_gyr[2];

        // cout<<"raw gyr0: "<<imuData.gX<<" rect gyr0: "<<rect_gyr[0]<<endl;
        // cout<<"raw gyr1: "<<imuData.gY<<" rect gyr1: "<<rect_gyr[1]<<endl;
        // cout<<"raw gyr2: "<<imuData.gZ<<" rect gyr2: "<<rect_gyr[2]<<endl;


        
 
        // imu.linear_acceleration.x = imuData.aX;
        // imu.linear_acceleration.y = imuData.aY;
        // imu.linear_acceleration.z = imuData.aZ;
        //  tmp_acc<<imuData.aX, imuData.aY, imuData.aZ;

        tmp_acc<<-imuData.aZ, -imuData.aY, -imuData.aX;
        rect_acc=acc_TK*(tmp_acc-acc_bias);
        // rect_acc=tmp_acc; // use orignal measurement 

        rect_acc-=acc_bias_after_rect;
        // cout<<"raw acc0: "<<imuData.aX<<" rect acc0: "<<rect_acc[0]<<endl;
        // cout<<"raw acc1: "<<imuData.aY<<" rect acc1: "<<rect_acc[1]<<endl;
        // cout<<"raw acc2: "<<imuData.aZ<<" rect acc2: "<<rect_acc[2]<<endl;

        // imu.linear_acceleration.x = rect_acc[0];
        // imu.linear_acceleration.y = rect_acc[1];
        // imu.linear_acceleration.z = rect_acc[2];
        
        // if(cnt<4000){
        //     mean_rect_acc2.push_back(rect_acc[2]);
        //     mean_rect_acc1.push_back(rect_acc[1]);
        //     mean_rect_acc0.push_back(rect_acc[0]);
        //     cnt++;
        // }
        // else {
          
        //     double mean_acc2=accumulate(mean_rect_acc2.begin(),mean_rect_acc2.end(),0.0)*1.0/mean_rect_acc2.size();
        //     double mean_acc1=accumulate(mean_rect_acc1.begin(),mean_rect_acc1.end(),0.0)*1.0/mean_rect_acc1.size();
        //     double mean_acc0=accumulate(mean_rect_acc0.begin(),mean_rect_acc0.end(),0.0)*1.0/mean_rect_acc0.size();
        //     cout<<"mean acc2: "<<mean_acc2<< " mean acc1: "<<mean_acc1<< "mean acc0: "<<mean_acc0<<endl;
        //     cnt=0;
        //     mean_rect_acc2.clear();
        //     mean_rect_acc1.clear();
        // }

        // imu.linear_acceleration.x = -rect_acc[2];
        // imu.linear_acceleration.y = -rect_acc[1];
        // imu.linear_acceleration.z =- rect_acc[0];

        imu.linear_acceleration.x = rect_acc[0];
        imu.linear_acceleration.y = rect_acc[1];
        imu.linear_acceleration.z = rect_acc[2];

        sensor_pub.publish(imu);
    }
}


void odom_acc_Callback(const nav_msgs::Odometry::ConstPtr& nav_msg,
                                                     const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& acc_msg){
        filtered_imu_msg.header=nav_msg->header;
        filtered_imu_msg.angular_velocity=nav_msg->twist.twist.angular;
        filtered_imu_msg.linear_acceleration=acc_msg->accel.accel.linear;
        float imu_angular_cov[9];
    

        int j=0;
        for(int i=0;i<9; i ++){
            j=21+ i + int(i/3)*3;
            filtered_imu_msg.angular_velocity_covariance[i]=nav_msg->twist.covariance[j];
            // cout<<"cov j1: "<<j <<endl;
        }
        

        j=0;
        for(int i=0;i<9; i ++){
            j=i + int(i/3)*3;
            filtered_imu_msg.linear_acceleration_covariance[i]=acc_msg->accel.covariance[j];
            // cout<<"cov j2: "<<j <<endl;
        }
        
        imu_filtered_pub.publish(filtered_imu_msg);

        



}

void correctFramerate(int resolution)
{
    switch (resolution)
    {
    case 0:
        gFps = sl_oc::video::FPS::FPS_15;
        gResolution = sl_oc::video::RESOLUTION::HD2K;
        break;
    case 1:
        gFps = sl_oc::video::FPS::FPS_30;
        gResolution = sl_oc::video::RESOLUTION::HD1080;
        break;
    case 2:
        gFps = sl_oc::video::FPS::FPS_60;
        gResolution = sl_oc::video::RESOLUTION::HD720;
        break;
    case 3:
        gFps = sl_oc::video::FPS::FPS_30;
        gResolution = sl_oc::video::RESOLUTION::VGA;
        break;
    default:
        ROS_FATAL("Unknow resolution passed");
        return;
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
    imu_filtered_pub = nh.advertise<sensor_msgs::Imu>("imu/filtered", 200);

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odometry/filtered", 100);
    message_filters::Subscriber<geometry_msgs::AccelWithCovarianceStamped> acc_sub(nh, "/accel/filtered", 100);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::AccelWithCovarianceStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, acc_sub);
    // sync.registerCallback(boost::bind(&odom_acc_Callback, _1, _2));
  

    ros::Timer image_timer;
    ros::Timer sensor_timer;
    
    
    cout<<"acc_inv: "<<endl;
    cout<<acc_inv_TK.matrix()<<endl;

    cout<<"gyr_inv: "<<endl;
    cout<<gyr_inv_TK.matrix()<<endl;


    // get ros param
    int resolution;
    private_nh.param("resolution", resolution, 3);
    resolution=3;
    std::cout<<"Resulution: "<<resolution<<std::endl;
    correctFramerate(resolution);
    zed = new StereoCamera(gResolution, gFps);
    if (zed->camera)
    {
        image_timer = nh.createTimer(ros::Duration(0.001), image_callback);
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
