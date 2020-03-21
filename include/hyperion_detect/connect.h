#include <hyperion_msgs/TrackedPersons.h>
//#include <hyperion_detect/detect.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class Connect {
private: // variables
  ros::NodeHandle *nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Time *timestamp_;
  
  std::vector<std::vector<std::vector<double>>> buffer_; // write down structure somewhere

  //Detect det_;

public: // constructor
  Connect(ros::NodeHandle *nh, ros::Time *current_time);

private: // internal functions
  void read(const sensor_msgs::PointCloud2::ConstPtr &msg);
  geometry_msgs::Quaternion angleToQuaternion(const double theta);
  geometry_msgs::Quaternion toQuaternionMsg(const Eigen::Quaternionf& quaternion);
  hyperion_msgs::TrackedPersons vect2msg(std::vector<std::vector<double>> q_people);

public: // external functions
  void publish();
  void pseudo_pub(std::vector<std::vector<double>> buf);
  void to_buffer(std::vector<std::vector<double>> item);
};
