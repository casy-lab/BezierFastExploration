/* Bezier Exploration                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: February 2022                         */
/* File: map_evaluator.cpp                     */

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float64MultiArray.h>

std::shared_ptr<octomap::OcTree> ot_;
ros::Time receivedTime;
bool inComputation = false;
void octomapCallback(const octomap_msgs::Octomap& msg){
    if(inComputation)
        return;

    octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
    octomap::OcTree* ot = (octomap::OcTree*)aot;
    ot_ = std::make_shared<octomap::OcTree>(*ot);
    receivedTime = ros::Time::now();

    delete ot;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "map_evaluator");
    ros::NodeHandle nh("~");

    ros::Subscriber octomapSubscriber = nh.subscribe("/octomap_full", 1, &octomapCallback);
    ros::Publisher statePublisher = nh.advertise<std_msgs::Float64MultiArray>("/exploration_state", 1);

    ros::Rate rate(0.1);

    while(ros::ok()){
        if(ot_){
            inComputation = true;
            ot_->expand();
            double res = ot_->getResolution();
            int n = ot_->calcNumNodes();

            std_msgs::Float64MultiArray state;
            state.data.push_back(receivedTime.toSec());
            state.data.push_back((double)n*res*res*res);
            statePublisher.publish(state);

            inComputation = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}