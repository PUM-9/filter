#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include "include/Filter.h"

namespace fs = boost::filesystem;

int main(int argc, char** argv) {
    if (argc == 1) {
        std::cout << "Usage: filter sources" << std::endl;
        std::cout << "Where sources is a list of files or directories." << std::endl;
        return 0;
    }

    Filter filter;

    for (int i = 1; i < argc; ++i) {
        fs::path p(argv[i]);

        if (fs::exists(p)) {
            if (fs::is_regular_file(p)) {
                PointCloud::Ptr cloud_in(new PointCloud), cloud_out(new PointCloud);

                if (pcl::io::loadPCDFile(p.native(), *cloud_in) == -1) {
                    PCL_ERROR("Couldn't read cloud file\n");
                    return -1;
                }

                filter.filter(cloud_in, cloud_out);

                std::string filename = p.stem().string() + "_filtered.pcd";
                pcl::io::savePCDFile(filename, *cloud_out);

            } else if (fs::is_directory(p)) {

            }
        }
    }


    /*
    pcl::visualization::PCLVisualizer viewer;
    viewer.addCoordinateSystem(10);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 200, 25, 25);
    viewer.addPointCloud(cloud_in, cloud_in_color, "cl_in");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in2_color(cloud_in2, 25, 200, 25);
    viewer.addPointCloud(cloud_in2, cloud_in2_color, "cl_in2");

    viewer.spin();
    */
}
