#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in cloud data
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    // Generate Data
    for (auto& point: *cloud)
    {
        point.x = 1024*rand() / (RAND_MAX + 1.0f);
        point.y = 1024*rand() / (RAND_MAX + 1.0f);
        point.z = 1.0;
    }
    // Set outliers
    (*cloud)[0].z = 2.0;
    (*cloud)[3].z = -2.0;
    (*cloud)[6].z = 4.0;

    std::cerr << "Point cloud data: " << cloud->size() << " points " << std::endl;
    for(const auto& point: *cloud)
    {
        std::cerr << "    "<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
    }
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create Seg obj
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffs);
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Not planes found.\n");
        return -1;
    }
    std::cerr << "Model coeffs: "<<coeffs->values[0]<<" "<<coeffs->values[1]<<" "<<coeffs->values[2]<< " "<<coeffs->values[3]<<std::endl; 
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                                << cloud->points[idx].y << " "
                                << cloud->points[idx].z << std::endl;
    return (0);
}