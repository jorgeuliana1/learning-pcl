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

        protected:
            PointCloud<PointXYZ>::Ptr cloud;
    };
};

#endif /* MY_POINT_CLOUD_HPP */