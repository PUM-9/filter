#ifndef FILTER_FILTER_H
#define FILTER_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Filter {
    private:
        float scaling_factor = 0.5f;

        int x_min = 310, x_max = 568;
        int y_min = 250, y_max = 580;
        int z_min = -1, z_max = 1;

        void move_to_origin();
        bool remove_stick(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, bool inverse = false) const;

    public:
        void filter(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, int rotation = 0) const;


        float get_scaling_factor() const { return scaling_factor; }
        void set_scaling_factor(float scaling_factor) { Filter::scaling_factor = scaling_factor; }
};


#endif //FILTER_FILTER_H
