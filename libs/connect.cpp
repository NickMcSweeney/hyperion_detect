// std libs
#include <iostream>
#include <math.h>
#include <string>
#include <utility>
#include <vector>

// ros libs
#include <ros/ros.h>

// hyperion libs
#include <hyperion_detect/connect.h>
#include <hyperion_msgs/TrackedPerson.h>
#include <hyperion_msgs/TrackedPersons.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

Connect::Connect(ros::NodeHandle *node, ros::Time *current_time) {
  this->nh_ = node;
  this->timestamp_ = current_time;

  vector<vector<vector<double>>> *buff_ptr = &(this->buffer_);
  // this->det_ = Detect(buff_ptr);

  this->pub_ = (*node).advertise<hyperion_msgs::TrackedPersons>(
      "/hyperion/detections", 1000);
  this->sub_ =
      (*node).subscribe("/camera/depth/points", 1000, &Connect::read, this);
}

void Connect::publish() {
  // publish detected position.
  try {
    // convert this->buffer_ to TrackedPerson;
    if (this->buffer_.empty())
      throw;
    vector<vector<double>> dat = this->buffer_.back();
    hyperion_msgs::TrackedPersons tracks_msg = vect2msg(dat);
    // publish
    this->pub_.publish(tracks_msg);
  } catch (const char *err) {
    cerr << err << endl;
  }
  this->buffer_.clear();
  return;
}

void Connect::read(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // read in sensor data from the kinect camera
  // convert to pcl::PointCloud<T>
  // PointCloudT pcl_cloud;
  // pcl::fromROSMsg(msg, *pcl_cloud);
  // call detection function to get people data
  // this->det_.find_people(pcl_cloud);
  // add to buffer
  return;
}

void Connect::pseudo_pub(vector<vector<double>> buf) {
  // publish position from file.
  try {
    // convert this->buffer_ to TrackedPerson;
    if (buf.empty())
      throw "empty buffer, no new readings!";
    hyperion_msgs::TrackedPersons tracks_msg = vect2msg(buf);
    // publish
    this->pub_.publish(tracks_msg);
  } catch (const char *err) {
    cerr << err << endl;
  }
  this->buffer_.clear();
  return;
}

hyperion_msgs::TrackedPersons
Connect::vect2msg(vector<vector<double>> q_people) {

  if (q_people.size() < 1) {
    throw "less than 1 person in queue";
  }

  hyperion_msgs::TrackedPersons tracked_people;
  std_msgs::Header head;
  // head.stamp = *(this->timestamp_);
  head.stamp = ros::Time::now();
  tracked_people.header = head;

  for (const auto &agent_state : q_people) {
    hyperion_msgs::TrackedPerson person;
    person.track_id = agent_state[0];
    person.is_occluded = false;
    person.detection_id = agent_state[0];

    const double theta = atan2(agent_state[4], agent_state[3]);
    geometry_msgs::PoseWithCovariance pose_with_cov;
    pose_with_cov.pose.position.x = agent_state[1];
    pose_with_cov.pose.position.y = agent_state[2];
    pose_with_cov.pose.position.z = 0;
    pose_with_cov.pose.orientation = angleToQuaternion(theta);
    person.pose = pose_with_cov;

    geometry_msgs::TwistWithCovariance twist_with_cov;
    twist_with_cov.twist.linear.x = agent_state[3];
    twist_with_cov.twist.linear.y = agent_state[4];
    person.twist = twist_with_cov;

    tracked_people.tracks.push_back(person);
  }
  return tracked_people;
}

geometry_msgs::Quaternion Connect::angleToQuaternion(const double theta) {
  Eigen::Matrix3f rotation_matrix(Eigen::Matrix3f::Identity());
  rotation_matrix(0, 0) = cos(theta);
  rotation_matrix(0, 1) = -sin(theta);
  rotation_matrix(0, 2) = 0;
  rotation_matrix(1, 0) = sin(theta);
  rotation_matrix(1, 1) = cos(theta);
  rotation_matrix(1, 2) = 0;
  rotation_matrix(2, 0) = 0;
  rotation_matrix(2, 1) = 0;
  rotation_matrix(2, 2) = 1;

  Eigen::Quaternionf quaternion(rotation_matrix);
  return toQuaternionMsg(quaternion.normalized());
}

geometry_msgs::Quaternion
Connect::toQuaternionMsg(const Eigen::Quaternionf &quaternion) {
  geometry_msgs::Quaternion gq;
  gq.x = quaternion.x();
  gq.y = quaternion.y();
  gq.z = quaternion.z();
  gq.w = quaternion.w();
  return move(gq);
}

void Connect::to_buffer(vector<vector<double>> item) {
  this->buffer_.push_back(item);
}
