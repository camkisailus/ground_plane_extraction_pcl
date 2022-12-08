#include <iostream>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/sample_consensus/method_types.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/features/normal_3d.h>

int main()
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read("/home/cuhsailus/Desktop/Research/22_academic_year/surface_normals/voxel_pcd/1659742380.933621644.pcd", *cloud_blob);
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        //Estimate normals
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(cloud_p);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.03);
        ne.compute(*cloud_normals);
        pcl::PointCloud<pcl::PointNormal> pn_cloud;
        pcl::concatenateFields(*cloud_p, *cloud_normals, pn_cloud);
        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
        writer.write<pcl::PointNormal> (ss.str (), pn_cloud, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }

    return (0);
}
