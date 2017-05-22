# filter
A small program for filtering point clouds produced by [TreeD](https://gitlab.ida.liu.se/tddd96-grupp9 "TreeD GitLab") so they can be used with the main 3DCopy software.

It was the result of a project in the course TDDD96 at Linköping University in 2017.

## Usage

The program provides a command line interface. Running it with the `-h` flag shows a usage:

```
Usage: filter [options] sources

sources is a list of files and directories with .pcd files to filter

Allowed options:
  -h [ --help ]         produce help message
  -r [ --rotation ] arg rotate the point cloud to a rotation
  -c [ --curve ] arg    rotate the point cloud to a curve
  --cutoff_height arg   the cutoff height in mm for the filter
                        use lower values for objects that are higher up
                        default=10
  -s [ --scale ] arg    the scaling factor used to adjust for different cart
                        speeds
                        the default works for cart speed=200
                        default=0.5
```

## Installation

#### Install script
A script is provided for installing the filter on a Ubuntu system. To install the filter on Ubuntu, run these commands:

```
git clone https://github.com/PUM-9/filter.git
cd filter
./install.sh
```

#### Build it yourself
If you're not using Ubuntu or if for some other reason the scipt doesn't work for you, you need to build it yourself. Its dependencies are [Boost](http://www.boost.org/ Boost) and [PCL](https://www.pointclouds.org/ "Point Cloud Library"). You also need [CMake](http://www.cmake.org CMake) and the usual dependencies for building C++. When all dependencies are met, run:

```
git clone https://github.com/PUM-9/filter.git
cd filter
mkdir build
cd build
cmake ..
make
make install
```

## License

This program is licensed under the GNU Lesser General Public License 3.
