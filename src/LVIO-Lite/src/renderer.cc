

#include <map>
#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <execution>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


#include "lvio_lite/KeyFrame.h"


using namespace std;
// **********************************************************************
// PCL alias
using PointT = pcl::PointXYZI;
using PointN = pcl::PointXYZINormal;
using PointC = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudN = pcl::PointCloud<PointN>;
using PointCloudC = pcl::PointCloud<PointC>;
using CloudTPtr = PointCloudT::Ptr;
using CloudNPtr = PointCloudN::Ptr;
using CloudCPtr = PointCloudC::Ptr;
using PointTVec = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
using PointNVec = std::vector<PointN, Eigen::aligned_allocator<PointN>>;
using PointCVec = std::vector<PointC, Eigen::aligned_allocator<PointC>>;
// **********************************************************************


// **********************************************************************
// vision 
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                        643.638019, 0.000000, 641.997757,
                        0.000000, 643.636015, 364.077191,
                        0.000000, 0.000000, 1.000000);
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) <<-0.046611, 
                                                0.038856, 
                                                -0.000602, 
                                                0.000277, 
                                                0.0);
Eigen::Matrix3f cameraMatrix_eigen;

cv::Mat m_ud_map1, m_ud_map2;
// vision
// **********************************************************************


// **********************************************************************
Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity();
string Topic1, Topic2;
void ReadParam(ros::NodeHandle& nh){
  vector<float> TL2C_tran;
  vector<float> TL2C_rot;
  Eigen::Vector3f trans;
  Eigen::Matrix3f R_rot;

  nh.param<string>("/CameraTopic",   Topic1, "/camera/color/image_raw");
  nh.param<string>("/KeyframeTopic", Topic2, "/lvio/keyframe");
  nh.param<vector<float>>("/TL2C_tran", TL2C_tran, vector<float>());
  nh.param<vector<float>>("/TL2C_rot",  TL2C_rot, vector<float>());

  trans[0] = TL2C_tran[0];
  trans[1] = TL2C_tran[1];
  trans[2] = TL2C_tran[2];
  R_rot = Eigen::Map<const Eigen::Matrix<float, -1, -1, 
                     Eigen::RowMajor>>(TL2C_rot.data(), 3, 3);

  extrinsic.block<3,3>(0,0) = R_rot;
  extrinsic.block<3,1>(0,3) = trans;

  cameraMatrix_eigen << 643.638019, 0.000000,   641.997757,
                        0.000000,   643.636015, 364.077191,
                        0.000000,   0.000000,   1.000000;

  cv::initUndistortRectifyMap( cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size(1280, 720),
                               CV_16SC2, m_ud_map1, m_ud_map2 );
}
// **********************************************************************



void AddColorToPointCloud(
    const CloudTPtr cloud,              // 输入点云
    const cv::Mat &image,               // 输入图像
    const Eigen::Matrix4f &extrinsic,   // 雷达到相机的外参
    const Eigen::Matrix3f &intrinsic,   // 相机的内参
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud)   // 输出：有颜色的点云
{
  colored_cloud->clear();
  colored_cloud->points.reserve(cloud->points.size());

  for (const auto &point : cloud->points)
  {
    Eigen::Vector4f transformed_point = extrinsic * Eigen::Vector4f(point.x, point.y, point.z, 1);
    float x = -transformed_point[1] / transformed_point[0];
    float y = -transformed_point[2] / transformed_point[0];
    Eigen::Vector3f pixel = intrinsic * Eigen::Vector3f(x, y, 1);

    int px = static_cast<int>(pixel[0]);
    int py = static_cast<int>(pixel[1]);

    if (px >= 0 && px < image.cols && py >= 0 && py < image.rows)
    {
      cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(px, py));
      pcl::PointXYZRGB colored_point;
      colored_point.x = point.x;
      colored_point.y = point.y;
      colored_point.z = point.z;
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->points.push_back(colored_point);
    }
  }
}



// **********************************************************************
// vision 
void image_equalize(cv::Mat &img, int amp)
{
  cv::Mat img_temp;
  cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
  clahe->apply(img, img_temp);
  img = img_temp;
}

cv::Mat equalize_color_image_Ycrcb(cv::Mat &image)
{
  cv::Mat hist_equalized_image;
  cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);
  std::vector<cv::Mat> vec_channels;
  cv::split(hist_equalized_image, vec_channels);
  image_equalize( vec_channels[0], 1 );
  cv::merge(vec_channels, hist_equalized_image);
  cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
  return hist_equalized_image;
}
// vision
// **********************************************************************


struct Cloud6D_t{
  float time;
  CloudTPtr cloud;
  Eigen::Vector3f pose_t;
  Eigen::Quaternionf pose_q;
};

struct CV_Mat_t{
  float time;
  cv::Mat img;
};

ros::Publisher pubColor, pubRaw;
std::deque<std::pair<Cloud6D_t, CV_Mat_t>> lv_vec;

void LV_Handler(const lvio_lite::KeyFrameConstPtr &lidar, const sensor_msgs::ImageConstPtr &vision){
  // lidar
  Cloud6D_t this_cloud;
  this_cloud.time = lidar->header.stamp.toSec();
  this_cloud.pose_t = {lidar->position.x, lidar->position.y, lidar->position.z};
  this_cloud.pose_q = Eigen::Quaternionf(lidar->quaternion.w, lidar->quaternion.x, lidar->quaternion.y, lidar->quaternion.z);
  this_cloud.cloud.reset(new PointCloudT());
  pcl::fromROSMsg(lidar->cloud, *(this_cloud.cloud));
  
  // vision
  CV_Mat_t this_img;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*vision, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;
  cv::Mat image_undistort;
  cv::remap( image, image_undistort, m_ud_map1, m_ud_map2, cv::INTER_LINEAR );
  this_img.img = equalize_color_image_Ycrcb(image_undistort);
  this_img.time = vision->header.stamp.toSec();
  
  // save to vector
  lv_vec.push_back(std::make_pair(this_cloud, this_img));

  if(1){
    cv::imshow("image", image);
    cv::imshow("image_undistort", image_undistort);
    cv::imshow("image_undistort_equalize", this_img.img);
    cv::waitKey(1);
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "lidar2vision");
  ros::NodeHandle nh;
  ReadParam(nh);

  pubColor = nh.advertise<sensor_msgs::PointCloud2>("/lvio/color", 1);
  pubRaw = nh.advertise<sensor_msgs::PointCloud2>("/lvio/raw", 1);  

  message_filters::Subscriber<lvio_lite::KeyFrame> *mf_lidar;
  message_filters::Subscriber<sensor_msgs::Image> *mf_vision;

  typedef message_filters::sync_policies::ApproximateTime<lvio_lite::KeyFrame, sensor_msgs::Image> VisionLidarPolicy;
  message_filters::Synchronizer<VisionLidarPolicy> *lidar_vision_sync;


  mf_lidar = new message_filters::Subscriber<lvio_lite::KeyFrame>(nh, Topic2, 100, ros::TransportHints().tcpNoDelay());
  mf_vision = new message_filters::Subscriber<sensor_msgs::Image>(nh, Topic1, 100, ros::TransportHints().tcpNoDelay());

  lidar_vision_sync = new message_filters::Synchronizer<VisionLidarPolicy>(VisionLidarPolicy(100), *mf_lidar, *mf_vision);
  lidar_vision_sync->registerCallback(boost::bind(&LV_Handler, _1, _2));

  ros::Rate loop(500);
  bool init = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pc(new pcl::PointCloud<pcl::PointXYZRGB>());

  while(ros::ok()){
    ros::spinOnce();
    if(lv_vec.empty()){
      loop.sleep();
      continue;
    }

    if(!init){
      auto& data = lv_vec.front();
      AddColorToPointCloud(data.first.cloud, data.second.img, extrinsic, cameraMatrix_eigen, color_pc);
      if(!color_pc->empty()){
        Eigen::Matrix4f interpolated_pose = Eigen::Matrix4f::Identity();
        interpolated_pose.block<3,3>(0,0) = data.first.pose_q.toRotationMatrix();
        interpolated_pose.block<3,1>(0,3) = data.first.pose_t;
        pcl::transformPointCloud(*color_pc, *color_pc, interpolated_pose);
        sensor_msgs::PointCloud2 color_pc_msg;
        pcl::toROSMsg(*color_pc, color_pc_msg);
        color_pc_msg.header.frame_id = "map";
        color_pc_msg.header.stamp = ros::Time().fromSec(data.second.time);
        pubColor.publish(color_pc_msg);
      }
      init = true;
      loop.sleep();
      continue;
    }
    
    if(lv_vec.size() < 3){
      loop.sleep();
      continue;
    }

    auto& data0 = lv_vec[0];
    auto& data1 = lv_vec[1];
    auto& data2 = lv_vec[2];


    // Eigen::Vector3f interpolated_position;
    // Eigen::Quaternionf interpolated_quaternion;

    // if(data1.first.time < data1.second.time){
    //   float alpha = (data1.second.time - data1.first.time ) / (data2.first.time - data1.first.time);
    //   interpolated_position = data1.first.pose_t + alpha * (data2.first.pose_t - data1.first.pose_t);
    //   interpolated_quaternion = data1.first.pose_q.slerp(alpha, data2.first.pose_q);
    // }else{
    //   float alpha = (data1.second.time - data0.first.time ) / (data1.first.time - data0.first.time);
    //   interpolated_position = data0.first.pose_t + alpha * (data1.first.pose_t - data0.first.pose_t);
    //   interpolated_quaternion = data0.first.pose_q.slerp(alpha, data1.first.pose_q);
    // }

    // Eigen::Matrix4f interpolated_pose = Eigen::Matrix4f::Identity();
    // interpolated_pose.block<3,3>(0,0) = interpolated_quaternion.toRotationMatrix();
    // interpolated_pose.block<3,1>(0,3) = interpolated_position;

    // Eigen::Matrix4f relative_pose = Eigen::Matrix4f::Identity();
    // interpolated_pose.block<3,3>(0,0) = (data1.first.pose_q.inverse()*interpolated_quaternion).toRotationMatrix();
    // interpolated_pose.block<3,1>(0,3) = data1.first.pose_q.inverse()*interpolated_position - data1.first.pose_t;

    // CloudTPtr refine_cloud(new PointCloudT());
    // pcl::transformPointCloud(*(data1.first.cloud), *refine_cloud, relative_pose);


    CloudTPtr refine_cloud(new PointCloudT());
    refine_cloud = data1.first.cloud;
    Eigen::Matrix4f interpolated_pose = Eigen::Matrix4f::Identity();
    interpolated_pose.block<3,3>(0,0) = data1.first.pose_q.toRotationMatrix();
    interpolated_pose.block<3,1>(0,3) = data1.first.pose_t; 

    AddColorToPointCloud(refine_cloud, data1.second.img, extrinsic, cameraMatrix_eigen, color_pc);
    
    if(!color_pc->empty()){
      pcl::transformPointCloud(*color_pc, *color_pc, interpolated_pose);
      sensor_msgs::PointCloud2 color_pc_msg;
      pcl::toROSMsg(*color_pc, color_pc_msg);
      color_pc_msg.header.frame_id = "map";
      color_pc_msg.header.stamp = ros::Time().fromSec(data1.second.time);
      pubColor.publish(color_pc_msg);
    }

    lv_vec.pop_front();
    
    loop.sleep();
  }

  return 0;
}
