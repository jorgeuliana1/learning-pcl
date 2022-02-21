#include <iostream>
#include <cmath>

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

#define PI 3.14159265

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
    apply_transformation(offset, PointXYZ(1, 1, 1), PointXYZ(0, 0, 0));
}

void MyPCL::MyPointCloud::apply_scale(double x_s, double y_s, double z_s) {
    apply_transformation(PointXYZ(0, 0, 0), PointXYZ(x_s, y_s, z_s), PointXYZ(0, 0, 0));
}

void MyPCL::MyPointCloud::apply_scale(double scale) {
    apply_scale(scale, scale, scale);
}

void MyPCL::MyPointCloud::apply_rotation(double x_r, double y_r, double z_r) {
    apply_transformation(PointXYZ(0, 0, 0), PointXYZ(1, 1, 1), PointXYZ(x_r, y_r, z_r));
}

void MyPCL::MyPointCloud::apply_transformation(PointXYZ offset, PointXYZ scale, PointXYZ rotation) {
    Matrix4d m_x, m_y, m_z;
    m_x = Matrix4d::Identity();
    m_y = Matrix4d::Identity();
    m_z = Matrix4d::Identity();

    auto to_rad = [] (double degree) {
        return degree / 180 * PI;
    };

    double rad_x, rad_y, rad_z;
    rad_x = to_rad(rotation.x);
    rad_y = to_rad(rotation.y);
    rad_z = to_rad(rotation.z);

    m_x(1, 1) = cos(rad_x);
    m_x(1, 2) = -sin(rad_x);
    m_x(2, 1) = sin(rad_x);
    m_x(2, 2) = cos(rad_x);

    m_y(0, 0) = cos(rad_y);
    m_y(2, 0) = sin(rad_y);
    m_y(0, 2) = -sin(rad_y);
    m_y(2, 2) = cos(rad_y);

    m_z(0, 0) = cos(rad_z);
    m_z(0, 1) = -sin(rad_z);
    m_z(1, 0) = sin(rad_z);
    m_z(1, 1) = cos(rad_z);

    Matrix4d m_rotation;
    m_rotation = m_x * m_y * m_z;

    Matrix4d m_scale;
    m_scale = Matrix4d::Identity();

    // Setting the scaling matrix
    /* 
     * x_s 0   0   0
     * 0   y_s 0   0
     * 0   0   z_s 0
     * 0   0   0   1
     */

    m_scale(0, 0) = scale.x;
    m_scale(1, 1) = scale.y;
    m_scale(2, 2) = scale.z;

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

    PointCloud<PointXYZ>::Ptr transformed;
    transformed.reset(new PointCloud<PointXYZ>());

    Matrix4d m_transform;
    m_transform = m_scale * m_rotation * m_translation;

    transformPointCloud(*cloud, *transformed, m_transform);
    cloud = transformed;
}