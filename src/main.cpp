#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>

#include "include/Filter.h"

namespace fs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char** argv) {
    // Declare the supported options
    po::options_description cli_options("Allowed options");
    cli_options.add_options()
            ("help,h", "produce help message")
            ("cutoff_height,c", po::value<int>(), "the cutoff height in mm for the filter\n"
                    "use lower values for objects that are higher up\n"
                    "default=10")
            ("scale,s", po::value<float>(), "the scaling factor used to adjust for different cart speeds\n"
                    "the default works for cart speed=200\n"
                    "default=0.5");

    po::options_description hidden("Hidden options");
    hidden.add_options()
            ("sources", po::value<std::vector<std::string> >(), "input files");

    po::options_description all("All options");
    all.add(cli_options).add(hidden);

    po::positional_options_description positional;
    positional.add("sources", -1);

    // Parse the cli arguments
    po::variables_map vm;
    po::store(po::command_line_parser(argc, (const char *const *)argv).options(all).positional(positional).allow_unregistered().run(), vm);
    po::notify(vm);

    // Print help text
    if (vm.count("help") || !vm.count("sources")) {
        std::cout << "Usage: filter [options] sources" << std::endl << std::endl;
        std::cout << "sources is a list of files and directories with .pcd files to filter" << std::endl << std::endl;
        std::cout << cli_options << std::endl;
        return 1;
    }

    Filter filter;

    // Set the cutoff height of the filter to input
    if (vm.count("cutoff_height")) {
        filter.set_cluster_cutoff(vm["cutoff_height"].as<int>());
    }

    // Set the scaling factor of the filter
    if (vm.count("scale")) {
        filter.set_scaling_factor(vm["scale"].as<float>());
    }

    // Get a vector of all the input sources
    std::vector<std::string> sources = vm["sources"].as<std::vector<std::string> >();

    // and go through them all
    for (int i = 0; i < sources.size(); ++i) {
        fs::path p(sources[i]);

        if (fs::exists(p)) {
            if (fs::is_regular_file(p) && p.extension().string() == ".pcd") {
                PointCloud::Ptr cloud_in(new PointCloud), cloud_out(new PointCloud);

                if (pcl::io::loadPCDFile(p.string(), *cloud_in) == -1) {
                    PCL_ERROR("Couldn't read cloud file\n");
                    return -1;
                }

                filter.filter(cloud_in, cloud_out);

                std::string filename = p.stem().string() + "_filtered.pcd";
                std::cout << "Saving filtered cloud to " << filename << std::endl;
                pcl::io::savePCDFile(filename, *cloud_out);

            } else if (fs::is_directory(p)) {
                for (auto it = fs::directory_iterator(p); it != fs::directory_iterator(); ++it) {
                  if (fs::is_regular_file(it->path()) && it->path().extension() == ".pcd") {
                        PointCloud::Ptr cloud_in(new PointCloud), cloud_out(new PointCloud);

                        if (pcl::io::loadPCDFile(it->path().string(), *cloud_in) == -1) {
                            PCL_ERROR("Couldn't read cloud file\n");
                            return -1;
                        }

                        filter.filter(cloud_in, cloud_out);

                        std::string filename = p.remove_trailing_separator().string() + "_filtered/"
                                               + it->path().filename().string();
                        fs::create_directory(fs::path(filename).parent_path());

                        std::cout << "Saving filtered cloud to " << filename << std::endl;
                        pcl::io::savePCDFile(filename, *cloud_out);
                    }
                }
            }
        }
    }
}
