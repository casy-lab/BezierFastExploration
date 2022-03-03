/* Bezier Exploration                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: February 2022                         */
/* File: bezier_exploration_node.cpp           */

#include <bezier_exploration/bezier_exploration.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "bezier_exploration_node");
  ros::NodeHandle nh("~");

  BezierExplorer explorer(nh);
  ros::spin();

  return 0;
}