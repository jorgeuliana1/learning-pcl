#include <iostream>

using ::std::string;

#include <vector>

using ::std::vector;

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

using ::pcl::PointCloud;
using ::pcl::PointXYZ;
using ::pcl::computeCentroid;
using ::pcl::transformPointCloud;
using ::pcl::io::loadPLYFile;
using ::pcl::visualization::CloudViewer;

#include <Eigen/Dense>

using ::Eigen::Matrix4d;

#include "my_point_cloud.hpp"

MyPCL::MyPointCloud::MyPointCloud(string path) {
    int pcl_load_status;

    cloud.reset(new PointCloud<PointXYZ>());
    pcl_load_status = loadPLYFile(path, *cloud);

    if (pcl_load_status == -1) {
        PCL_ERROR("Reading error!\n");
        exit(pcl_load_status);
    }
}

MyPCL::MyPointCloud::~MyPointCloud() {}

void MyPCL::MyPointCloud::view() {
    CloudViewer viewer("My PointCloud viewer");

    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {} // Busy waiting
}

PointXYZ MyPCL::MyPointCloud::centroid() {
    size_t centroid_dimension;
    PointXYZ centroid;

    centroid_dimension = computeCentroid(*cloud, centroid);

    if (centroid_dimension < 3)
        exit(-1);

    return centroid;
}

void MyPCL::MyPointCloud::apply_offset(PointXYZ offset) {
    Matrix4d m_translation;
    m_translation = Matrix4d::Identity();

    // Setting the translation matrix
    /* 
     * 1 0 0 x
     * 0 1 0 y
     * 0 0 1 z
     * 0 0 0 1
     */
    m_translation(0, 3) = offset.x;
    m_translation(1, 3) = offset.y;
    m_translation(2, 3) = offset.z;

    PointCloud<PointXYZ>::Ptr translated;
    translated.reset(new PointCloud<PointXYZ>());

    transformPointCloud(*cloud, *translated, m_translation);
    cloud = translated;
}