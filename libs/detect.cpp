// std libs
#include <string>
#include <vector>

// ros libs
#include <ros/ros.h>

// hyperion libs
#include <hyperion_detect/detect.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

Detect::Detect(vector<vector<vector<double>>> *buffer) {
	//this->buffer_ = buffer;
}


void Detect::find_people(PointCloudT pc) {
// do some sort of detection 
}
