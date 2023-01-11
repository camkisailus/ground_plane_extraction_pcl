#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include "surface_normals/PlaneArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/sample_consensus/method_types.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/features/normal_3d_omp.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/crop_box.h>
#include <pcl-1.8/pcl/filters/project_inliers.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

class ExtractPlanesNode{

    private:
    int counter_;
    bool process_;
    ros::Publisher cloud_pub_;
    ros::Publisher hull_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher poly_pub_;
    ros::Subscriber cloud_sub_;
    pcl::VoxelGrid<pcl::PCLPointCloud2> downsample_;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac_from_norm_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_;
    pcl::ExtractIndices<pcl::Normal> extract_normal_indices_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator_;
    pcl::ProjectInliers<pcl::PointXYZ> project_inliers_;
    tf::TransformListener listener_;


    public:
    ExtractPlanesNode(ros::NodeHandle *nh)
    {
        cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>("kisailus_plane", 1);
        hull_pub_ = nh->advertise<sensor_msgs::PointCloud2>("kisailus_plane_hull", 1);
        plane_pub_ = nh->advertise<surface_normals::PlaneArray>("plane_fit", 1);
        poly_pub_ = nh->advertise<geometry_msgs::PolygonStamped>("poly", 1);
        std::string cloud_topic;
        nh->getParam("cloud_topic", cloud_topic);
        cloud_sub_ = nh->subscribe(cloud_topic, 1, &ExtractPlanesNode::cloud_cb, this);
        sac_from_norm_.setOptimizeCoefficients(true);
        sac_from_norm_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        float normal_distance_weight;
        nh->getParam("normal_distance_weight", normal_distance_weight);
        sac_from_norm_.setNormalDistanceWeight(normal_distance_weight);
        sac_from_norm_.setMethodType(pcl::SAC_RANSAC);
        sac_from_norm_.setMaxIterations(1000);
        sac_from_norm_.setDistanceThreshold(0.03);
        project_inliers_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        process_ = true;
    }
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);

};
