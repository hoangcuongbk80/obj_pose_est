#include <ros/package.h>
#include<vector>

// OpenCV specific includes
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
//plane fitting
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/geometry.h>

#ifndef MODEL_TO_SCENE_H
#define MODEL_TO_SCENE_H

struct boundingBBox
{
    double length[3];
    pcl::PointXYZ minPoint, maxPoint; //min max Point after trasferring to origin
    pcl::PointXYZ center;
    Eigen::Matrix4f toOrigin; //pcl::transformPointCloud(input, output, toOrigin);
    pcl::PointCloud<pcl::PointXYZ> cornerPoints;

};

struct ObjectModel
{
    std::string name;
	  pcl::PointCloud<pcl::PointXYZ> points;
	  pcl::PointCloud<pcl::PointXYZRGB> colorPoints;
    boundingBBox OBB;   
};

class model_to_scene
{
  public:
    model_to_scene();
    virtual ~model_to_scene();

    int                    numOfObjects; // Number of objects in database
    int                    vfh_tolerance, vfh_subRange;
    std::string            *objectName;
    std::vector<int>       recognizedObjects;

    int img_height, img_width;
    cv::Mat *depthImg, *rgbImg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr OBB;
    pcl::PointCloud<pcl::PointXYZ>::Ptr database_cloud;
    std::vector<int> PointIndices;
    std::vector<std::vector<int> > ClusterIndices;

    void computeOBB(const pcl::PointCloud<pcl::PointXYZ> &input, boundingBBox &OBB);
    void coarseToFineRegistration(const pcl::PointCloud<pcl::PointXYZ> &scene, 
                                  const pcl::PointCloud<pcl::PointXYZ> &model);
};

#endif