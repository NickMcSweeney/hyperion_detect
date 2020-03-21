#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class Detect {
	private:
	// variables
	std::vector<std::vector<std::vector<double>>> *buffer_;
	public: 
	// constructor
	Detect(std::vector<std::vector<std::vector<double>>> *buffer);
	private:
	// internal functions
	public:
	// public functions
	void find_people(pcl::PointCloud<pcl::PointXYZRGBA> pc);
};
