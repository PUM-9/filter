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

/**
 * Performs a clustering filter on the input cloud and centers the filtered cloud around
 * the origin
 * @param cloud_in The input cloud to be filtered
 * @param cloud_out The output filtered cloud
 */
void Filter::filter(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out) const {
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

    return;
}

/**
 * Moves the input cloud to the origin by looking at the stick.
 * Also moves the cloud down so that the lowest point is at height 0.
 * @param cloud_in The input point cloud.
 * @param cloud_out The resulting transformed point cloud.
 */
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

/**
 * Finds and removes the stick holding the object in the scanner by dividing the
 * input cloud into clusters and removing all clusters below a certain threshold.
 * Can also return just the stick by passing inverse = true.
 * @param cloud_in The input point cloud to be filtered.
 * @param cloud_out The resulting point cloud without the stick.
 * @param inverse Filter out the object and return the stick instead.
 * @return true if successful
 */
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

        // Set data for the point cloud
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    if (!clusters.empty()) {
        std::vector<PointCloud::Ptr> good_clusters;

        // Go through all clusters and find the ones we are looking for
        for (size_t i = 0; i < clusters.size(); ++i) {
            Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
            pcl::compute3DCentroid(*clusters.at(i), centroid);

            if (!inverse && centroid(0, 0) < cluster_cutoff) {
                good_clusters.push_back(clusters[i]);
            } else if (inverse && centroid(0, 0) >= cluster_cutoff) {
                good_clusters.push_back(clusters[i]);
            }
        }

        // Add the good clusters to cloud_out
        for (size_t i = 0; i < good_clusters.size(); ++i) {
            if (i == 0) {
                *cloud_out = *good_clusters.at(i);
            } else {
                *cloud_out += *good_clusters.at(i);
            }
        }

        return true;
    } else {
        // We could not find any clusters, return the input cloud
        *cloud_out = *cloud_in;
        return false;
    }
}
