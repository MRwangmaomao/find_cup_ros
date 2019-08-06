/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <XmlRpcException.h>
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
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h> 

  
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

#include "find_cup_ros/fast_find_ellipse_detector.h"

#include "image_geometry/pinhole_camera_model.h"

using namespace Eigen;
using namespace cv;

cv_bridge::CvImagePtr cv_image_; // 接收到的原始图像
Eigen::Matrix3d camera_K_; // 相机内参
cv::Mat src_; // 接收到的原始图像Mat格式
double beta_; // 低通滤波因子
bool is_continue_; // 是否连续接收图片
double cup_height_; // 杯子高度
std::shared_ptr<image_transport::ImageTransport> it_; 
image_transport::CameraSubscriber camera_image_subscriber_; // 原始图像接收
ros::Publisher cup_detections_publisher_; // 发布杯子pose
ros::Subscriber apriltags_subscriber_; // 接收apriltag消息
cv::Point2i last_center_; // 上一时刻杯子中心
int tick_num_debug = 0; // 控制打印速度

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
 * @param src 原始图像
 * @param m_src_array 四个二维码的中心像素坐标
 * @param temp_q 二维码的姿态
 * @param cup_height 杯子高度
 * @param K 相机内参
 */
void find_cup_in_four_apriltag(const  cv::Mat & src, std::map<int ,cv::Point2f > m_src_array, std::map<int ,Eigen::Vector3d >m_translation, Eigen::Quaterniond temp_q, const double cup_height, Eigen::Matrix3d K)
{ 
    Mat img_canny;
    Mat img_thresh;
    Mat img_blur;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    bool find_cup_flag = false; 
    cv::Mat image = src.clone();
    // cv::resize(image, image, cv::Size(src.cols/6, src.rows/6), 0, 0, cv::INTER_LINEAR);
    // 1.计算二维码的法向量
    ROS_DEBUG_STREAM(m_translation[0](0) << " " << m_translation[0](1) << " " << m_translation[0](2));
    Eigen::Vector3d v1(m_translation[0](0) - m_translation[1](0), m_translation[0](1) - m_translation[1](1), m_translation[0](2) - m_translation[1](2));
    Eigen::Vector3d v2(m_translation[0](0) - m_translation[2](0), m_translation[0](1) - m_translation[2](1), m_translation[0](2) - m_translation[2](2));
    Eigen::Vector3d v_dir = v1.cross(v2);//  .normalize(); //二维码的方向向量
    Eigen::Vector3d v_step = v_dir * (cup_height/v_dir.norm());
    ROS_DEBUG_STREAM("移动向量距离：" << v_step(0) << "  "  << v_step(1) << "  "  << v_step(2));
    
    // 2.将二维码的空间坐标沿着法向量移动杯子高度的距离后投影到像素平面
    cv::Point2f virtual_points[4];
    for(int i = 0; i < 4; i++)
    {
        Eigen::Vector3d temp = K * (m_translation[i] - v_step);
        Eigen::Vector3d temp_v = m_translation[i] - v_step;
        ROS_DEBUG_STREAM("沿着法线平移后的空间坐标：" << temp_v(0) << "  " <<  temp_v(1) << "  " <<  temp_v(2));
        virtual_points[i] = cv::Point2d(temp(0)/temp(2), temp(1)/temp(2)); 
        ROS_DEBUG_STREAM("投影到像素平面的坐标为：" << temp(0)/temp(2) << "  " <<  temp(1)/temp(2));
    } 
    
    // 3.求两个平面的射影变换矩阵H
    cv::Point2f dest_points[4];
    for(int i = 0; i < 4; i++)
    {
        dest_points[i] = m_src_array[i];
        // ROS_DEBUG_STREAM("投影到像素平面的坐标为" << m_src_array[i].x << "  " <<  m_src_array[i].y);
    }
    cv::Mat H = cv::getPerspectiveTransform(virtual_points, dest_points); 
    // ROS_DEBUG_STREAM("输出射影变换矩阵" << H.dims);
      
    // 4.射影变换(连接一些连通域)  
    warpPerspective( src , image, H, image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    
    

    // 5. 提取图像ROI
    // std::vector<std::vector<cv::Point>>  roi_contours;
    // std::vector<cv::Point> roi_countour;
    // roi_countour.push_back(dest_points[0]);
    // roi_countour.push_back(dest_points[1]);
    // roi_countour.push_back(dest_points[3]);
    // roi_countour.push_back(dest_points[2]);
    // roi_contours.push_back(roi_countour);
    // cv::Mat roi_mask = Mat::zeros(src.size(), CV_8UC1);
    // cv::drawContours(roi_mask, roi_contours, -1, cv::Scalar::all(255),CV_FILLED);
    // cv::Mat maskImage;
    // image.copyTo(maskImage, roi_mask);
    // cv::namedWindow("maskRegion", CV_WINDOW_AUTOSIZE );
    // cv::imshow("maskRegion",maskImage);
    // waitKey(3);

    // 6. 找椭圆
    cv::Point2i cup_center_point = OnImage(image);
    if(cup_center_point.x != 0 && cup_center_point.y != 0)
    {
        find_cup_flag = true;
        // 7. 低通滤波
        if(last_center_.x == 0 && last_center_.y == 0)
        {
            
        } 
        else
        {
            cup_center_point.x = last_center_.x + cvRound((cup_center_point.x - last_center_.x) * beta_);
            cup_center_point.y = last_center_.y + cvRound((cup_center_point.y - last_center_.y) * beta_);
        }
        cv::circle(image, cup_center_point, 3, (255,0,255),4);
        cv::circle(src, cup_center_point, 3, (255,0,255),4);
        
        // 8. 据杯子的图像中心坐标点计算相机在真实世界中的坐标
        if(find_cup_flag)
        {   
            
            Eigen::Matrix2d a; 
            a << m_src_array[1].x - m_src_array[0].x, m_src_array[2].x - m_src_array[0].x, m_src_array[1].y - m_src_array[0].y ,m_src_array[2].y - m_src_array[0].y;
            Eigen::Vector2d b(cup_center_point.x - m_src_array[0].x, cup_center_point.y - m_src_array[0].y);
            Eigen::Vector2d c = a.colPivHouseholderQr().solve(b); 
            Eigen::Vector3d d( m_translation[0] + (m_translation[1]-m_translation[0])*c(0) + (m_translation[2]-m_translation[0])*c(1));
            ROS_DEBUG_STREAM("输出三维坐标" << d);
            // std::cout << "OK";
            geometry_msgs::Pose cup_pose;
            //ROS发布杯子的三维坐标消息
            cup_pose.position.x = d(0);
            cup_pose.position.y = d(1);
            cup_pose.position.z = d(2);
            cup_pose.orientation.x = temp_q.x();
            cup_pose.orientation.y = temp_q.y();
            cup_pose.orientation.z = temp_q.z();
            cup_pose.orientation.w = temp_q.w();
            // ROS_DEBUG_STREAM("x,y");
            cup_detections_publisher_.publish(cup_pose); 
        } 

        // namedWindow("imshow_image", CV_WINDOW_AUTOSIZE );
        // imshow("imshow_image",image);
        // cvWaitKey(3); 
        namedWindow("imshow_src", CV_WINDOW_AUTOSIZE );
        imshow("imshow_src",src);
        cvWaitKey(3); 
        last_center_ = cup_center_point; // 更新中心值 
    } 
}
 

/**
 * @brief 接收AprilTag消息回调函数
 * 
 * @param tag_detections 
 */
void apriltagsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_detections)
{ 
    // 存储四个二维码的map
    std::map<int ,cv::Point2f >m_src_points;
    // 存储四个二维码空间位姿的map
    std::map<int ,Eigen::Vector3d >m_trans;
 
    if (tag_detections->detections.size() == 0)
    {
        tick_num_debug++;
        if(tick_num_debug > 80)
        { 
            tick_num_debug = 0;
            ROS_WARN_STREAM("No detected tags!");
        }
    }
    else if (tag_detections->detections.size() != 4)
    {
        tick_num_debug++;
        if(tick_num_debug > 80)
        { 
            tick_num_debug = 0;
            ROS_WARN_STREAM("detected is not 4 tags!");
        }
    }
    else // 检测到四个二维码
    {
      Eigen::Quaterniond temp_q;
      ROS_DEBUG_STREAM("detected tags nums is " << tag_detections->detections.size());
      for(int i = 0; i < tag_detections->detections.size(); i++)
      { 
          // ROS_DEBUG_STREAM("detected tags " << tag_detections->detections[i].id[0]);
          // ROS_DEBUG_STREAM("detected position " << tag_detections->detections[i].center_point[0] << "," << tag_detections->detections[i].center_point[1]);
          m_src_points[tag_detections->detections[i].id[0]] = cv::Point2f(cv::Point(tag_detections->detections[i].center_point[0],tag_detections->detections[i].center_point[1]));
          geometry_msgs::PoseWithCovariance tag_pose = tag_detections->detections[i].pose.pose;  
          temp_q.x() = tag_pose.pose.orientation.x;
          temp_q.y() = tag_pose.pose.orientation.y;
          temp_q.z() = tag_pose.pose.orientation.z;
          temp_q.w() = tag_pose.pose.orientation.w;
          Eigen::Vector3d temp_trans(tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z); 
          m_trans[tag_detections->detections[i].id[0]] = temp_trans; 
          // ROS_DEBUG_STREAM("Camera detect tag is " << tag_detections->detections[i].id[0] << "  " << tag_pose.pose.position.x << " "  << tag_pose.pose.position.y << " "  << tag_pose.pose.position.z);
          // v_trans.push_back(temp_trans);
      } 
      find_cup_in_four_apriltag(src_, m_src_points, m_trans, temp_q, cup_height_ , camera_K_);
    }  
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
    double fx = camera_model.fx(); // focal length in camera x-direction [px]
    double fy = camera_model.fy(); // focal length in camera y-direction [px]
    double cx = camera_model.cx(); // optical center x-coordinate [px]
    double cy = camera_model.cy(); // optical center y-coordinate [px]
    // double fx = 610.3236694335938;  
    // double fy = 610.5026245117188; 
    // double cx = 313.3859558105469; 
    // double cy = 237.25076293945312; 

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
        src_ = cv::imread("/home/wpr/find_cup222.jpeg"); // 修改读取图片的路径
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
  
//   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &imageCallback );

  apriltags_subscriber_ =
      nh.subscribe("/tag_detections", 100,
                          &apriltagsCallback );

  cup_detections_publisher_ =       
      nh.advertise<geometry_msgs::Pose>("/cup_detections", 1);

  is_continue_ = getAprilTagOption<bool>(pnh, 
      "is_continue", true); 

  cup_height_ = getAprilTagOption<double>(pnh, 
      "cup_height", 0.0); 

  beta_ = getAprilTagOption<double>(pnh, 
      "beta", 0.2); 


  ros::spin();
  return 0;
}
