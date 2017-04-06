#include "include/Filter.h"
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void Filter::filter(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, int rotation, int curve) const {
    // Use a pass through filter to remove all points outside of specific coordinates
    pcl::PassThrough<pcl::PointXYZ> pt_filter;
    pt_filter.setInputCloud(cloud_in);

    // Do initial rough filtering on the x axis to remove noise data
    pt_filter.setFilterFieldName("x");
    pt_filter.setFilterLimits(x_min, x_max);
    pt_filter.setFilterLimitsNegative(false);
    pt_filter.filter(*cloud_out);

    // Filter on the z axis to remove the plane of noise data in the
    // beginning of the scan
    pt_filter.setInputCloud(cloud_out);
    pt_filter.setFilterFieldName("z");
    pt_filter.setFilterLimits(z_min, z_max);
    pt_filter.setFilterLimitsNegative(true);
    pt_filter.filter(*cloud_out);

    // Filter on the y axis
    pt_filter.setInputCloud(cloud_out);
    pt_filter.setFilterFieldName("y");
    pt_filter.setFilterLimits(y_min, y_max);
    pt_filter.setFilterLimitsNegative(false);
    pt_filter.filter(*cloud_out);

    // Remove points that are far away from other points
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud_out);
    outlier_filter.setRadiusSearch(0.8);
    outlier_filter.setMinNeighborsInRadius(2);
    outlier_filter.filter(*cloud_out);

    // Move the object to the origin of the coordinate system
    move_to_origin(cloud_out, cloud_out);

    // Remove the stick the object is attached to
    remove_stick(cloud_out, cloud_out);

    // Scale the point cloud to make it the correct proportions
    Eigen::Affine3f scale_transform(Eigen::Affine3f::Identity());
    scale_transform.scale(Eigen::Vector3f(1, 1, scaling_factor));
    pcl::transformPointCloud(*cloud_out, *cloud_out, scale_transform);

    // Rotate the object around the x axis to match the objects real world rotation
    Eigen::Matrix3f rotation_matrix(Eigen::AngleAxisf((rotation*M_PI) / 180, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f rotation_transform(Eigen::Affine3f::Identity());
    rotation_transform.rotate(rotation_matrix);
    pcl::transformPointCloud(*cloud_out, *cloud_out, rotation_transform);

    return;
}

void Filter::move_to_origin(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out) const {
    // Translate the object to move the center of the object to the origin (approximately)
    Eigen::Affine3f translation_transform(Eigen::Affine3f::Identity());
    translation_transform.translation() << -538.0, -346.0, 591.0;
    pcl::transformPointCloud(*cloud_in, *cloud_out, translation_transform);

    // Extract the stick holding the object and compute it's centroid (center of mass)
    PointCloud::Ptr stick(new PointCloud);
    remove_stick(cloud_in, stick, true);
    Eigen::Vector4f stick_centroid(Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*stick, stick_centroid);

    // Find the lowest point of the object to lay it flat on the surface
    PointCloud::Ptr without_stick(new PointCloud);
    remove_stick(cloud_in, without_stick);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*without_stick, min, max);

    // Adjust the position of the object so the stick is in the center
    // This means the object is in the origin.
    translation_transform.translation() << -max.x, -stick_centroid(1, 0), -stick_centroid(2, 0);
    pcl::transformPointCloud(*cloud_out, *cloud_out, translation_transform);
}

bool Filter::remove_stick(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, bool inverse) const {
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    search_tree->setInputCloud(cloud_in);

    // Create the object for extracting cluster in cloud_in
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setInputCloud(cloud_in);

    // Set the maximum distance between two points in a cluster to 4 mm
    cluster_extraction.setClusterTolerance(4.0);

    // Set a cluster to be between 10 and 1000000 points
    cluster_extraction.setMinClusterSize(10);
    cluster_extraction.setMaxClusterSize(1000000);

    // Perform the euclidean cluster extraction algorithm
    cluster_extraction.setSearchMethod(search_tree);
    cluster_extraction.extract(cluster_indices);

    // Generate all the individual cluster point clouds
    std::vector<PointCloud::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cluster(new PointCloud);

        // For every list of indices, add all the points to a point cloud
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloud_in->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    if (!clusters.empty()) {
        std::vector<PointCloud::Ptr> good_clusters;

        for (size_t i = 0; i < clusters.size(); ++i) {
            Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
            pcl::compute3DCentroid(*clusters.at(i), centroid);

            std::cout << "Centroid: " << std::endl << centroid << std::endl << std::endl;

            if (!inverse && centroid(0, 0) < cluster_x_max) {
                good_clusters.push_back(clusters[i]);
            } else if (inverse && centroid(0, 0) >= cluster_x_max) {
                good_clusters.push_back(clusters[i]);
            }
        }

        std::cout << "Total clusters: " << clusters.size() << std::endl;
        std::cout << "Good clusters: " << good_clusters.size() << std::endl;

        for (size_t i = 0; i < good_clusters.size(); ++i) {
            if (i == 0) {
                *cloud_out = *good_clusters.at(i);
            } else {
                *cloud_out += *good_clusters.at(i);
            }
        }

        return true;
    } else {
        *cloud_out = *cloud_in;
        return false;
    }
}
