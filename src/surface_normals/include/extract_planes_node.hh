#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
    ros::Publisher cloud_filtered1_pub_;
    ros::Publisher cloud_filtered2_pub_;
    ros::Subscriber cloud_sub_;
    pcl::VoxelGrid<pcl::PCLPointCloud2> downsample_;
    // pcl::SACSegmentation<pcl::PointXYZ> seg_;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac_from_norm_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_;
    pcl::ExtractIndices<pcl::Normal> extract_normal_indices_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator_;
    pcl::ProjectInliers<pcl::PointXYZ> project_inliers_;
    tf::TransformListener listener_;
    // pcl::PCDWriter writer_;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;


    public:
    ExtractPlanesNode(ros::NodeHandle *nh)
    {
        cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>("kisalus_plane_output", 1);
        cloud_filtered1_pub_ = nh->advertise<sensor_msgs::PointCloud2>("kisalus_filtered1_plane_output", 1);
        cloud_filtered2_pub_ = nh->advertise<sensor_msgs::PointCloud2>("kisalus_filtered2_plane_output", 1);
        std::string cloud_topic;
        nh->getParam("cloud_topic", cloud_topic);
        cloud_sub_ = nh->subscribe(cloud_topic, 1, &ExtractPlanesNode::cloud_cb, this);
        // seg_.setOptimizeCoefficients(true);
        // seg_.setModelType(pcl::SACMODEL_PLANE);
        // seg_.setMethodType(pcl::SAC_RANSAC);
        // seg_.setMaxIterations(1000);
        // seg_.setDistanceThreshold(0.05);
        sac_from_norm_.setOptimizeCoefficients(true);
        sac_from_norm_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        float normal_distance_weight;
        nh->getParam("normal_distance_weight", normal_distance_weight);
        sac_from_norm_.setNormalDistanceWeight(normal_distance_weight);
        sac_from_norm_.setMethodType(pcl::SAC_RANSAC);
        sac_from_norm_.setMaxIterations(1000);
        sac_from_norm_.setDistanceThreshold(0.03);
        project_inliers_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        counter_ = 0;
        process_ = true;
    }
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);

};