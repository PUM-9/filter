#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

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

                if (pcl::io::loadPCDFile(p.string(), *cloud_in) == -1) {
                    PCL_ERROR("Couldn't read cloud file\n");
                    return -1;
                }

                filter.filter(cloud_in, cloud_out, 0, 0);

                std::string filename = p.stem().string() + "_filtered.pcd";
                std::cout << "Saving to " << filename << std::endl;
                pcl::io::savePCDFile(filename, *cloud_out);

            } else if (fs::is_directory(p)) {
                for (auto it = fs::directory_iterator(p); it != fs::directory_iterator(); ++it) {
                    if (fs::is_regular_file(it->path())) {
                        PointCloud::Ptr cloud_in(new PointCloud), cloud_out(new PointCloud);

                        if (pcl::io::loadPCDFile(it->path().string(), *cloud_in) == -1) {
                            PCL_ERROR("Couldn't read cloud file\n");
                            return -1;
                        }

                        filter.filter(cloud_in, cloud_out, 0, 0);

                        std::string filename = p.string() + "_filtered/" + it->path().filename().string();
                        fs::create_directory(fs::path(filename).parent_path());

                        std::cout << "Saving to " << filename << std::endl;
                        pcl::io::savePCDFile(filename, *cloud_out);

                    }
                }
            }
        }
    }
}
