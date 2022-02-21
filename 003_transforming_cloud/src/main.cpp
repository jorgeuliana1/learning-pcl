/*
This program reads and shows a PLY file.
*/

#include <iostream>

using ::std::string;
using ::std::cout;
using ::std::endl;

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

using ::pcl::PointCloud;
using ::pcl::PointXYZ;
using ::pcl::io::loadPLYFile;
using ::pcl::visualization::CloudViewer;

#include "my_point_cloud.hpp"

using ::MyPCL::MyPointCloud;

// The asset used at this program was found at https://people.sc.fsu.edu/~jburkardt/data/ply/airplane.ply

/*
The asset path is passed at argv[1]
*/
int main(int argc, char **argv) {
    // Verifying argv
    if (argc < 2)
        return -1;

    // Getting the asset path
    string rel_path = argv[1];
    string ply_path = "../" + rel_path;
    
    // Loading the ply file
    MyPointCloud cloud(ply_path);

    // Getting the point cloud central coordinates
    PointXYZ centroid, negative_offset, zero(0, 0, 0);
    centroid = cloud.centroid();
    negative_offset.getArray3fMap() = zero.getArray3fMap() - centroid.getArray3fMap();
    // |-> negative_offset <- 0 - centroid = -centroid
    
    cloud.apply_transformation(
        negative_offset,                // Offset
        PointXYZ(0.001, 0.001, 0.001),  // Scale
        PointXYZ(180, 0, 0)             // Rotation
    );

    // Cloud view
    cloud.view();

    return 0;
}