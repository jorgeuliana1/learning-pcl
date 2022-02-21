#ifndef MY_POINT_CLOUD_HPP
#define MY_POINT_CLOUD_HPP

#include <vector>
#include <iostream>
#include <pcl/point_types.h>

namespace MyPCL {
    class MyPointCloud {
        public:
            MyPointCloud(std::string path);
            ~MyPointCloud();

            virtual void view();

            PointXYZ centroid();

            virtual void apply_offset(PointXYZ offset);

            virtual void apply_scale(double x_s, double y_s, double z_s);
            virtual void apply_scale(double scale);

            virtual void apply_rotation(double x_r, double y_r, double z_r);

            virtual void apply_transformation(PointXYZ offset, PointXYZ scale, PointXYZ rotation);

        protected:
            PointCloud<PointXYZ>::Ptr cloud;
    };
};

#endif /* MY_POINT_CLOUD_HPP */