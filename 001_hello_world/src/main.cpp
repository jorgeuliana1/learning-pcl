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
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    int pcl_load_status = loadPLYFile(ply_path, *cloud);
    if (pcl_load_status == -1) {
        PCL_ERROR("Reading error!\n");
        return pcl_load_status;
    }

    // Cloud view
    CloudViewer viewer("My first viewer!");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {} // Busy waiting

    return 0;
}