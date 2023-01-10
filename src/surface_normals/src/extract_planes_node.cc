// PCL Includes
#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include "extract_planes_node.hh"
#include "pcl-1.8/pcl/surface/convex_hull.h"
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#define PI 3.14159265


void ExtractPlanesNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{  
    sensor_msgs::PointCloud2* transformed = new sensor_msgs::PointCloud2;
    sensor_msgs::PointCloud2ConstPtr transformedPtr(transformed);
    if(!pcl_ros::transformPointCloud("camera_base", *input, *transformed, listener_)){
            ROS_ERROR("ERROR TRANSFORMING POINT CLOUD");
            return;
        }

    // ROS_WARN("GOT A MSG");
    if(process_)
    {
        // Convert incoming msg to useable PCL type
        pcl::PCLPointCloud2* in_cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr incloudPtr(in_cloud);
        pcl_conversions::toPCL(*transformedPtr, *in_cloud);
        // pcl_conversions::toPCL(*input, *in_cloud);

        // Transform cloud to inertial frame
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PCLPointCloud2ConstPtr cloudTransformedPtr(cloud_transformed);
        


        // std::stringstream ss;
        // ss << "/home/cuhsailus/Desktop/Research/22_academic_year/surface_normals/out_pcds/start.pcd";
        // writer_.write<pcl::PointXYZ> (ss.str(), *incloud, false);


        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PCLPointCloud2::Ptr cloud_cropped_blob (new pcl::PCLPointCloud2);
        // crop_.setInputCloud(incloudPtr);
        // crop_.filter(*cloud_cropped_blob);
        // pcl::fromPCLPointCloud2(*cloud_cropped_blob, *cloud_cropped);



        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        // downsample_.setInputCloud(cloud_transformed)
        downsample_.setInputCloud(incloudPtr);
        downsample_.setLeafSize(0.01f, 0.01f, 0.01f);
        pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
        downsample_.filter(*cloud_filtered_blob);
        pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

        
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        normal_estimator_.setSearchMethod(tree);
        normal_estimator_.setInputCloud(cloud_filtered);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_n (new pcl::PointCloud<pcl::Normal>);;
        normal_estimator_.setRadiusSearch(0.03);
        normal_estimator_.compute(*cloud_normals);
        int i = 0, nr_points = (int) cloud_filtered->size();
        std::vector<sensor_msgs::PointCloud2> clouds_to_publish;
        sensor_msgs::PointCloud2 cloud_out;
        sensor_msgs::PointCloud2 cloud_filtered_out;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
        surface_normals::PlaneArray plane_arr_msg;
        while(cloud_filtered->size() > 0.03*nr_points)
        {
            pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
            sac_from_norm_.setInputCloud(cloud_filtered);
            sac_from_norm_.setInputNormals(cloud_normals);
            sac_from_norm_.segment(*inliers, *coeffs);
            // ROS_WARN_STREAM("Coeffs[0]: "<< coeffs->values[0] << " Coeffs[1]: "<<coeffs->values[1]<< " Coeffs[2]: "<<coeffs->values[2] << " Coeffs[3]: "<< coeffs->values[3]);
            // std::string msg=;
            // ROS_WARN_STREAM(msg);
            Eigen::Vector3f normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
            float angle = (acos(Eigen::Vector3f::UnitZ().dot(normal))* 180.0 / PI);
            surface_normals::Plane plane_msg;
            plane_msg.header.frame_id = "camera_base";
            plane_msg.slope = angle;
            std::string color;
            if(i == 0){
                color = "red";
            }else{
                // break;
                color="purple";
            }
            ROS_WARN_STREAM("Iter : "<< i << +" Size: "<< inliers->indices.size() <<" Angle: "<<angle<< " Color: "<<color);

            // seg_.setInputCloud(cloud_filtered);
            // seg_.segment(*inliers, *coeffs);
            if(inliers->indices.size()==0){
                // ROS_WARN("Could not estimate planar model from given data");
                break;
            }
            // For RVIZ
            extract_indices_.setInputCloud(cloud_filtered);
            extract_indices_.setIndices(inliers);
            extract_indices_.setNegative(false);
            extract_indices_.filter(*cloud_p);
            pcl::toROSMsg(*cloud_p, cloud_out);
            cloud_pub_.publish(cloud_out);
            // For convex hull nodelet
            project_inliers_.setInputCloud(cloud_p);
            project_inliers_.setModelCoefficients(coeffs);
            project_inliers_.filter(*cloud_projected);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> chull;
            chull.setInputCloud (cloud_projected);
            // chull.setAlpha (0.1);
            chull.reconstruct (*cloud_hull);
            pcl::toROSMsg(*cloud_hull, cloud_filtered_out);
            cloud_filtered_pub_.publish(cloud_filtered_out);

            geometry_msgs::PolygonStamped p = geometry_msgs::PolygonStamped();
            p.header.frame_id = "camera_base";
            for(sensor_msgs::PointCloud2ConstIterator<float> it(cloud_filtered_out, "x"); it != it.end();++it)
            {
                geometry_msgs::Point32 pt;
                pt.x = it[0];
                pt.y = it[1];
                pt.z = it[2];
                plane_msg.hull_vertices.push_back(pt);
                p.polygon.points.push_back(pt);
            }
            poly_pub_.publish(p);
            plane_arr_msg.planes.push_back(plane_msg);
            // if(i == 0){
            //     cloud_filtered1_pub_.publish(cloud_filtered_out);
            // }else{
            //     cloud_filtered2_pub_.publish(cloud_filtered_out);
            // }
            // std::stringstream ss3;
            // ss3 << "/home/cuhsailus/Desktop/Research/22_academic_year/surface_normals/out_pcds/iter_"<<counter_<<"_" << i << ".pcd";
            // writer_.write<pcl::PointXYZ> (ss3.str(), *cloud_p, false);

            // Shrink clouds and iterate
            extract_indices_.setNegative(true);
            extract_indices_.filter(*cloud_f);
            cloud_filtered.swap(cloud_f);
            extract_normal_indices_.setInputCloud(cloud_normals);
            extract_normal_indices_.setIndices(inliers);
            extract_normal_indices_.setNegative(true);
            extract_normal_indices_.filter(*cloud_n);
            cloud_normals.swap(cloud_n);
            // normal_estimator_.setInputCloud(cloud_filtered);
            // normal_estimator_.compute(*cloud_normals);
            i++;
        }
        // ROS_WARN_STREAM("Done filtering cloud.. Ready to run py; i = " << i);
        plane_pub_.publish(plane_arr_msg);  
        counter_++;
        // process_ = false;
        // ROS_WARN("Finished processing full pointcloud");
    }
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "extract_normal_planes");
    ros::NodeHandle nh("plane_segmenter");
    ExtractPlanesNode foo(&nh);
    ros::spin();
    return 0;
}