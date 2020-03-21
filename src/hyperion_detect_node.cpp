/**
 * Hyperion RGB-d Vision integration with ROS
 * person detection module
 *
 */

#include <ros/ros.h>

#include "hyperion_detect/connect.h"

#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv) {
  /**
   * main function. init and connect withtthe ROS  system
   *
   */

  ros::init(argc, argv, "hyperion_detect");
  ros::NodeHandle nh;
  ros::Rate loop_rate(30); // publish 10 times per second

  ros::Time current_t = ros::Time::now();
  double pseudo_clock;
  double pdt = 0;
  double dt = 0;
  double count = 0;

  Connect conn = Connect(&nh, &current_t);
  ifstream pseudo_detect;
  pseudo_detect.open(
      "/home/usernamix/Workspace/src/hyperion/hyperion_data/atc-20121114.csv");
  vector<double> frozen;

  ROS_INFO("RUNNING");
  while (ros::ok()) {
    dt = ros::Time::now().toSec() - current_t.toSec();
    current_t = ros::Time::now();

    string line;
    bool initial = true;
    bool intime = true;
    bool is_open = pseudo_detect.is_open();
    if (!is_open)
      ROS_WARN("file not open!");

    vector<vector<double>> timeblock;

    // ROS_INFO("looping -->");
    while (intime && getline(pseudo_detect, line)) {
      // ROS_INFO("real time: %f", dt);
      // ROS_INFO("pseudo time: %f", pdt);

      stringstream sline(line);
      string elem;
      vector<double> rie;
      // read in data for each element in a line and put in a vector.
      while (getline(sline, elem, ',')) {
        rie.push_back(stod(elem));
      }
      if (rie.empty()) {
        ROS_ERROR("no data read in!");
        return 0;
      }

      if (!frozen.empty() && initial) {
        // ROS_INFO("INITAL READING - w/frozen");
        pdt = frozen[0] - pseudo_clock;
        pseudo_clock = frozen[0];
        initial = false;

        double r = frozen[5];
        double rth = frozen[6];
        double th = frozen[7];
        double vx = (r * cos(th) - rth * sin(th));
        double vy = (r * sin(th) + rth * cos(th));
        vector<double> temp = {frozen[1], frozen[2], frozen[3], vx, vy};
        timeblock.push_back(temp);
        frozen.clear();
      } else if (initial) {
        // ROS_INFO("INITAL READING");
        pdt = rie[0] - pseudo_clock;
        pseudo_clock = rie[0];
        initial = false;
      }

      // ROS_INFO("pclock: %f", pseudo_clock);

      if (pseudo_clock != rie[0]) {
        // ROS_INFO("FINAL READING");
        frozen = rie;
        intime = false;
      } else {
        cout << ".";
        double r = rie[5];
        double rth = rie[6];
        double th = rie[7];
        double vx = (r * cos(th) - rth * sin(th));
        double vy = (r * sin(th) + rth * cos(th));
        timeblock.push_back({rie[1], rie[2], rie[3], vx, vy});
      }
    }
    if (!timeblock.empty()) {
      conn.to_buffer(timeblock);
      timeblock.clear();
      cout << "\t\t";
    }

    count += dt;
    if (count > 0.046) {
      cout << endl;
      // conn.pseudo_pub(timeblock);
      ROS_INFO("publish [%f]", current_t.toSec());
      conn.publish();
      count = 0;
    }

    // conn.publish(); // publish msg every 1/10 second.

    ros::spinOnce();
    loop_rate.sleep();
  }
}
