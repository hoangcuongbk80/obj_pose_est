#include <iostream> 
#include <cstdlib> 
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>

#include "obj_pose_est/model_to_scene.h"
#include "obj_pose_est/AddTwoInts.h"

using namespace std;
using namespace cv;

cv::Mat label_img, depth_img;
double fx, fy, cx, cy, depth_factor;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

void depthToClould()
{
  depth_img = cv::imread("/home/hoang/temp/warehouse-results/0009/images/0000000001_depth.png", -1);
  label_img = cv::imread("/home/hoang/temp/warehouse-results/0009/images/0000000001_labels.png", -1);
  fx=581.52; fy=581.5; cx=319.0; cy=237.3; depth_factor=1000;
  scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB point;
  for(int row=0; row < depth_img.rows; row++)
  {
    for(int col=0; col < depth_img.cols; col++)       
    {
      if(isnan(depth_img.at<ushort>(row, col))) continue;
      double depth = depth_img.at<ushort>(row, col) / depth_factor;
      point.x = (col-cx) * depth / fx;
      point.y = (row-cy) * depth / fy;
      point.z = depth;
      point.b = 255;
      point.g = 255;
      point.r = 255;
      if(label_img.at<uchar>(row, col) == 6) scene_cloud->push_back(point);
    }
  }
}

bool add(obj_pose_est::AddTwoInts::Request  &req,
         obj_pose_est::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iliad_object_pose_estimation");

  ros::NodeHandle nh_, cloud_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("myCloud", 1);
  ros::Rate loop_rate(10);

  ros::ServiceClient client = nh_.serviceClient<obj_pose_est::AddTwoInts>("add_two_ints");
  obj_pose_est::AddTwoInts srv;
  srv.request.a = 13;
  srv.request.b = 3;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
/* 
  model_to_scene model2scene;
  std::string filename = "/home/hoang/temp/warehouse-results/sotstark_1.ply";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *cloud);
 */
  /* depthToClould();
  *cloud += *scene_cloud; */
  
  /* pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  cloud->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*cloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  
  while (ros::ok())
  {  
    cloud_pub.publish (output);
    ros::spinOnce();
    loop_rate.sleep();
  } */
  return 0;
}