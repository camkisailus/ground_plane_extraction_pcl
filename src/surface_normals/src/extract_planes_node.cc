// PCL Includes
#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include "extract_planes_node.hh"
#include "pcl-1.8/pcl/surface/convex_hull.h"
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cddlib/cdd.h>
#include <cddlib/cddmp.h>
#include <cddlib/setoper.h>
#define PI 3.14159265


void ExtractPlanesNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{  
    // Transform cloud to inertial frame
    sensor_msgs::PointCloud2* transformed = new sensor_msgs::PointCloud2;
    sensor_msgs::PointCloud2ConstPtr transformedPtr(transformed);
    if(!pcl_ros::transformPointCloud("camera_base", *input, *transformed, listener_)){
            ROS_ERROR("ERROR TRANSFORMING POINT CLOUD");
            return;
        }
    if(process_)
    {
        // Convert incoming msg to useable PCL type
        pcl::PCLPointCloud2* in_cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr incloudPtr(in_cloud);
        pcl_conversions::toPCL(*transformedPtr, *in_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
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
        sensor_msgs::PointCloud2 cloud_hull_out;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
        surface_normals::PlaneArray plane_arr_msg;
        while(cloud_filtered->size() > 0.03*nr_points)
        {
            // Segment plane
            pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
            sac_from_norm_.setInputCloud(cloud_filtered);
            sac_from_norm_.setInputNormals(cloud_normals);
            sac_from_norm_.segment(*inliers, *coeffs);
            if(inliers->indices.size()==0){
                // ROS_WARN("Could not estimate planar model from given data");
                break;
            }
            // Calculate Slope in Degrees
            Eigen::Vector3f normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
            float slope = (acos(Eigen::Vector3f::UnitZ().dot(normal))* 180.0 / PI);
            // Create Plane message
            surface_normals::Plane plane_msg;
            plane_msg.header.frame_id = "camera_base";
            plane_msg.slope = slope;
            // Publish segmented cloud for RVIZ
            extract_indices_.setInputCloud(cloud_filtered);
            extract_indices_.setIndices(inliers);
            extract_indices_.setNegative(false);
            extract_indices_.filter(*cloud_p);
            pcl::toROSMsg(*cloud_p, cloud_out);
            cloud_pub_.publish(cloud_out);
            // Find convex hull of plane
            project_inliers_.setInputCloud(cloud_p);
            project_inliers_.setModelCoefficients(coeffs);
            project_inliers_.filter(*cloud_projected);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> chull;
            chull.setInputCloud (cloud_projected);
            chull.reconstruct (*cloud_hull);
            pcl::toROSMsg(*cloud_hull, cloud_hull_out);
            // Publish vertices of convex hull
            hull_pub_.publish(cloud_hull_out);

            geometry_msgs::PolygonStamped p = geometry_msgs::PolygonStamped();
            p.header.frame_id = "camera_base";
            std::vector<std::vector<double>> vertices;
            for(sensor_msgs::PointCloud2ConstIterator<float> it(cloud_hull_out, "x"); it != it.end();++it)
            {
                geometry_msgs::Point32 pt;
                pt.x = it[0];
                pt.y = it[1];
                pt.z = it[2];
                plane_msg.hull_vertices.push_back(pt);
                p.polygon.points.push_back(pt);
                vertices.push_back({pt.x, pt.y, pt.z});
            }
            dd_set_global_constants();
            dd_MatrixPtr input_vertices = dd_CreateMatrix(4,2);
            dd_set_si(input_vertices->matrix[0][0], 1);
            dd_set_si(input_vertices->matrix[0][1], 1);
            dd_set_si(input_vertices->matrix[1][0], 1);
            dd_set_si(input_vertices->matrix[1][1], -1);
            dd_set_si(input_vertices->matrix[2][0], -1);
            dd_set_si(input_vertices->matrix[2][1], 1);
            dd_set_si(input_vertices->matrix[3][0], -1);
            dd_set_si(input_vertices->matrix[3][1], -1);
            // dd_MatrixPtr input_vertices = dd_CreateMatrix(vertices.size(), vertices[0].size());
            // for(int i = 0; i < vertices.size(); i++){
            //     for(int j = 0; j < vertices[i].size(); j++){
            //         dd_set_si(input_vertices->matrix[i][j], vertices[i][j]);
            //     }
            // }
            dd_ErrorType err = dd_NoError;
            dd_PolyhedraPtr poly = dd_DDMatrix2Poly(input_vertices, &err);
            if(err != dd_NoError){
                // Handle Error
                ROS_WARN_STREAM("Error in dd_DDMatrix2Poly");
            }else{
                dd_MatrixPtr A = dd_CopyInequalities(poly);
                dd_MatrixPtr G = dd_CopyGenerators(poly);
                ROS_WARN_STREAM("Inequalities");
                dd_WriteMatrix(stdout, A);
                ROS_WARN_STREAM("Generators");
                dd_WriteMatrix(stdout, G);
                ROS_WARN_STREAM("################################################");
            }
            dd_FreeMatrix(input_vertices);
            dd_FreePolyhedra(poly);
            // Publish polygon to RVIZ
            poly_pub_.publish(p);
            // Append plane msg to plane_arr msg
            plane_arr_msg.planes.push_back(plane_msg);

            // Shrink clouds and iterate
            extract_indices_.setNegative(true);
            extract_indices_.filter(*cloud_f);
            cloud_filtered.swap(cloud_f);
            extract_normal_indices_.setInputCloud(cloud_normals);
            extract_normal_indices_.setIndices(inliers);
            extract_normal_indices_.setNegative(true);
            extract_normal_indices_.filter(*cloud_n);
            cloud_normals.swap(cloud_n);
            i++;
        }
        plane_pub_.publish(plane_arr_msg); 
        dd_free_global_constants(); 
        process_ = false;
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