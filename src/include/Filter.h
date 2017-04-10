#ifndef FILTER_FILTER_H
#define FILTER_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Filter {
    private:
        // The scaling factor 0.5 assumes the scans are run with cart speed 200 mm/s.
        float scaling_factor = 0.5f;
        int cluster_cutoff = 10;

        void move_to_origin(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out) const;
        bool remove_stick(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, bool inverse = false) const;

    public:
        // Limits for the initial rough pass through filter
        int x_min = 310, x_max = 568;
        int y_min = 250, y_max = 580;
        int z_min = -1, z_max = 1;

        void filter(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out) const;

        // Functions for setting the scaling factor (for different cart speeds)
        float get_scaling_factor() const { return scaling_factor; }
        void set_scaling_factor(float scaling_factor) { Filter::scaling_factor = scaling_factor; }

        // Functions for setting the clusters height limit in mm
        // A lower value means the object is placed higher up
        int get_cluster_cutoff() const { return cluster_cutoff; }
        void set_cluster_cutoff(int cluster_x_max) { Filter::cluster_cutoff = cluster_x_max; }
};


#endif //FILTER_FILTER_H
