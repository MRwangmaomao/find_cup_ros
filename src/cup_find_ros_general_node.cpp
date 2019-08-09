 
#include <ros/ros.h>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h> 
#include <geometry_msgs/Pose.h>

#include "find_cup_ros/fast_find_ellipse_detector.h"
#include "image_geometry/pinhole_camera_model.h"

using namespace Eigen;
using namespace cv;

cv_bridge::CvImagePtr cv_image_; // 接收到的原始图像
Eigen::Matrix3d camera_K_; // 相机内参
cv::Mat src_; // 接收到的原始图像Mat格式 
bool is_continue_; // 是否连续接收图片
double cup_height_; // 杯子高度
double work_plane_height_; // 工作台到机械臂基座的高度
std::shared_ptr<image_transport::ImageTransport> it_; 
image_transport::CameraSubscriber camera_image_subscriber_; // 原始图像接收
ros::Publisher cup_detections_publisher_; // 发布杯子pose
ros::Subscriber camera2world_pose_subscriber_; // 接收相机到机械臂基座的变换 
Eigen::Matrix3d R_camera2world_; // 相机到机械臂基座的旋转矩阵 全局变量
Eigen::Vector3d t_camera2world_; // 相机到机械臂基座的平移向量 全局变量
bool is_get_pose_ = false; // 是否收到相机姿态消息
int tick_num_debug_ = 0; // 控制打印速度

/**
 * @brief Get the April Tag Option object 参数获取模板函数
 * 
 * @tparam T 
 * @param pnh 
 * @param param_name 
 * @param default_val 
 * @return T 
 */
template<typename T>
T getAprilTagOption(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}



/**
 * @brief 椭圆检测函数
 *  
 */
void find_cup(cv::Mat & src, Eigen::Matrix3d K)
{      
    cv::Point2i cup_center_point = OnImage(src);
    if(cup_center_point.x != 0 && cup_center_point.y != 0) //存在椭圆
    {     

        cv::circle(src, cup_center_point, 3, (255,0,255),4);
        std::cout << K(0,0) << " " << K(0,2) << " " << K(1,1) << " " << K(1,2) << endl;
        assert(K(0,0) != 0);
        Eigen::Vector3d ray_line_camera;
        ray_line_camera(0) = (cup_center_point.x - K(0,2))/K(0,0); 
        ray_line_camera(1) = (cup_center_point.y - K(1,2))/K(1,1);
        ray_line_camera(2) = 1.0;
        ROS_DEBUG_STREAM("cup_center_point.x: " << cup_center_point.x << "  cup_center_point.y: " << cup_center_point.y);
        ROS_DEBUG_STREAM("ray_line_camera: " << ray_line_camera); 
        
        Eigen::Vector3d d;
        Eigen::Vector3d ray_line_world = R_camera2world_ * ray_line_camera;
        ROS_DEBUG_STREAM("ray_line_world: " << ray_line_world); 
        
        double temp_scale = -(t_camera2world_(2) - work_plane_height_ - cup_height_)/ray_line_world(2);
        ROS_DEBUG_STREAM("temp_scale: " << temp_scale); 
 
        d(0) = t_camera2world_(0) + temp_scale * ray_line_world(0);
        d(1) = t_camera2world_(1) + temp_scale * ray_line_world(1);
        d(2) = work_plane_height_ + cup_height_;
        
        ROS_DEBUG_STREAM("输出三维坐标" << d); 
        geometry_msgs::Pose cup_pose;
        //ROS发布杯子的三维坐标消息
        cup_pose.position.x = d(0);
        cup_pose.position.y = d(1);
        cup_pose.position.z = d(2);
        cup_pose.orientation.x = 0;
        cup_pose.orientation.y = 0;
        cup_pose.orientation.z = 0;
        cup_pose.orientation.w = 1; 
        cup_detections_publisher_.publish(cup_pose);   

        namedWindow("src", CV_WINDOW_AUTOSIZE );
        imshow("src",src);
        cvWaitKey(3);  
    } 
}
 

/**
 * @brief 接收AprilTag消息回调函数
 * 
 * @param T_camera2world 
 */
void camera2worldPoseCallback(const geometry_msgs::Pose::ConstPtr& T_camera2world)
{ 
     
    t_camera2world_(0) = T_camera2world->position.x;
    t_camera2world_(1) = T_camera2world->position.y;
    t_camera2world_(2) = T_camera2world->position.z;

    Eigen::Quaterniond q_camera2world;
    q_camera2world.x() = T_camera2world->orientation.x;
    q_camera2world.y() = T_camera2world->orientation.y;
    q_camera2world.z() = T_camera2world->orientation.z;
    q_camera2world.w() = T_camera2world->orientation.w;

    R_camera2world_ = q_camera2world.toRotationMatrix();

    find_cup(src_, camera_K_);
}

/**
 * @brief 接收图像回调函数
 * 
 * @param image_rect 
 * @param camera_info 
 */
void imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{ 
    // 接收相机内参
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info);
    double fx, fy, cx, cy;
    if(is_continue_)
    { 
        // fx = camera_model.fx(); // focal length in camera x-direction [px]
        // fy = camera_model.fy(); // focal length in camera y-direction [px]
        // cx = camera_model.cx(); // optical center x-coordinate [px]
        // cy = camera_model.cy(); // optical center y-coordinate [px]
        fx = 608.8717041015625;  
        fy = 608.871826171875; 
        cx = 330.8108215332031; 
        cy = 232.7913055419922; 
    }
    else
    {
        fx = 610.3236694335938;  
        fy = 610.5026245117188; 
        cx = 313.3859558105469; 
        cy = 237.25076293945312; 
    }
     
    camera_K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1; 
    
    // 接收图像消息
    try
    {
        cv_image_ = cv_bridge::toCvCopy(image_rect,
                                    sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
 
    if(!is_continue_)
    {
        src_ = cv::imread("/home/wpr/1.png"); // 修改读取图片的路径
        cv_image_->image = src_.clone(); 
    }
    else    
        src_ = cv_image_->image.clone(); 
 
} 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_cup_ros");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
   
  ROS_INFO_STREAM("--------------------start find cup--------------------------");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &imageCallback );

  camera2world_pose_subscriber_ =
      nh.subscribe("/camera2world", 100,
                          &camera2worldPoseCallback );

  cup_detections_publisher_ =       
      nh.advertise<geometry_msgs::Pose>("/cup_detections", 1);

  is_continue_ = getAprilTagOption<bool>(pnh, 
      "is_continue", true); 

  cup_height_ = getAprilTagOption<double>(pnh, 
      "cup_height", 0.0); 
  
  work_plane_height_ = getAprilTagOption<double>(pnh, 
      "work_plane_height", 0.0); 

  ros::spin();
  return 0;
}
