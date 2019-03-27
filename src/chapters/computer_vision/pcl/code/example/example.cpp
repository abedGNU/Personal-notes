#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}