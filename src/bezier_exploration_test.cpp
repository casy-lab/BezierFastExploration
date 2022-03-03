/* Bezier Exploration                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: February 2022                         */
/* File: bezier_exploration_test.cpp           */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Eigen>

#include <bezier_exploration/InitService.h>
#include <bezier_exploration/BezierTraj.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Empty.h>

#include <visualization_msgs/Marker.h>

ros::Time startTime;

mavros_msgs::State current_state;
mavros_msgs::PositionTarget targetMsg;
bezier_exploration::InitService srv;
std_msgs::Empty exec;

Eigen::Matrix<double, 3, 1> actualPosition, lastSP;
double yawFromOdom = 0, lastYawSP;
uint explorationState = 0, execTraj = 2;

std::vector<Eigen::Matrix<double, 3, 6>> traj_nominal;
std::vector<double> ttTraj;
std::vector<double> yawTraj;

std::vector<Eigen::Matrix<double, 3, 6>> traj_safe;
std::vector<double> ttTraj_safe;

std::vector<double> yawVector = {0.0};
uint yawIndex = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    actualPosition(0) = msg->pose.pose.position.x;
    actualPosition(1) = msg->pose.pose.position.y;
    actualPosition(2) = msg->pose.pose.position.z;

    // Compute Yaw From Quaternion
    double roll = 0.0, pitch = 0.0;
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yawFromOdom);

    if(yawFromOdom < 0){
        yawFromOdom = 2*M_PI + yawFromOdom;
    }

    // Publish Odom Transformation
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(transformStamped);
}

void traj_cb(const bezier_exploration::BezierTraj::ConstPtr& msg){
    Eigen::Matrix<double, 3, 6> traj;
    Eigen::Matrix<double, 3, 6> safeTraj;

    for(uint ii = 0; ii < msg->x.size(); ii++){
        traj(0, ii) = msg->x[ii];
        traj(1, ii) = msg->y[ii];
        traj(2, ii) = msg->z[ii];
    }

    for(uint ii = 0; ii < msg->x_safe.size(); ii++){
        safeTraj(0, ii) = msg->x_safe[ii];
        safeTraj(1, ii) = msg->y_safe[ii];
        safeTraj(2, ii) = msg->z_safe[ii];
    }

    traj_nominal.push_back(traj);
    traj_safe.push_back(safeTraj);
    yawTraj.push_back(msg->yaw[msg->yaw.size()-1]);
    ttTraj.push_back(msg->time);
    ttTraj_safe.push_back(msg->time_safe);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bezier_exploration_test");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, odom_cb);
    ros::Subscriber traj_sub = nh.subscribe<bezier_exploration::BezierTraj>("/traj", 50, traj_cb);
    ros::Publisher target_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::Marker>("/traj_vis", 1);
    ros::Publisher exec_pub = nh.advertise<std_msgs::Empty>("/traj_executed", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient set_init = nh.serviceClient<bezier_exploration::InitService>("/init");

    ros::Rate rate(50.0);

    for(double vv = 0.5; vv <= 2*M_PI; vv += 0.5){
        yawVector.push_back(vv);
    }
    
    yawVector.push_back(2*M_PI);

    // Wait for FCU Connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    targetMsg.type_mask =   mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::IGNORE_VX  |
                            mavros_msgs::PositionTarget::IGNORE_VY  |
                            mavros_msgs::PositionTarget::IGNORE_VZ  |
                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    targetMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    targetMsg.header.stamp = ros::Time::now();
    targetMsg.position.x = 0.0;
    targetMsg.position.y = 0.0;
    targetMsg.position.z = 4.0;
    targetMsg.yaw = 0.0;

    // Send a Few Setpoints
    for(int i = 100; ros::ok() && i > 0; --i){
        target_pub.publish(targetMsg);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        Eigen::Matrix<double, 3, 6> actualTraj;
        double actualDelta = 1.0;
        double actualYaw = yawFromOdom;

        // Handle PX4 State Transition
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();

        }else {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }

                last_request = ros::Time::now();

            }
        }

        switch(explorationState){
        case 0:
            // Initializing
            startTime = ros::Time::now();
            
            targetMsg.position.x = 0.0;
            targetMsg.position.y = 0.0;
            targetMsg.position.z = 2.0;
            targetMsg.yaw = 0.0;

            lastSP = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 2.0);
            lastYawSP = 0.0;

            if(abs(actualPosition(2) - 2) < 1e-1){
                explorationState = 10;
            }

            break;

        case 10:
            // Initialize MAP
            startTime = ros::Time::now();

            if(yawIndex < yawVector.size()){
                targetMsg.position.x = 0.0;
                targetMsg.position.y = 0.0;
                targetMsg.position.z = 2.0;
                targetMsg.yaw = yawVector[yawIndex];

                if(abs(yawFromOdom - yawVector[yawIndex]) < 1e-3){
                    yawIndex ++;
                }
            } else{
                explorationState = 1;
            }            

            lastSP = Eigen::Matrix<double, 3, 1>(targetMsg.position.x, targetMsg.position.y, targetMsg.position.z);
            lastYawSP = targetMsg.yaw;

            break;
        
        case 1:
            // Start Exploration
            srv.request.x = actualPosition(0);
            srv.request.y = actualPosition(1);
            srv.request.z = actualPosition(2);
            srv.request.yaw = yawFromOdom;
            
            if(set_init.call(srv)){
                ROS_INFO("Start Exploration");
                explorationState = 2;
            }

            break;

        case 2:
            // Exploring
            double tt = (ros::Time::now() - startTime).toSec();

            switch(execTraj){
            case 0:
                // End Traj.
                if(tt > ttTraj[0]){
                    ttTraj.erase(ttTraj.begin());
                    traj_nominal.erase(traj_nominal.begin());
                    yawTraj.erase(yawTraj.begin());
                    startTime = ros::Time::now();

                    if(traj_nominal.size() != 0){
                        ttTraj_safe.erase(ttTraj_safe.begin());
                        traj_safe.erase(traj_safe.begin());
                    }

                    exec_pub.publish(exec);
                }

                // Check for New Piece
                if(traj_nominal.size() != 0){
                    actualTraj = traj_nominal[0];
                    actualDelta = ttTraj[0];
                    actualYaw = yawTraj[0];
                } else{
                    startTime = ros::Time::now();
                    execTraj = 1;
                }

                break;
            
            case 1:
                // End Safe Traj.
                if(tt > ttTraj_safe[0]){
                    ttTraj_safe.erase(ttTraj_safe.begin());
                    traj_safe.erase(traj_safe.begin());
                    startTime = ros::Time::now();
                    execTraj = 2;

                } else{
                    actualTraj = traj_safe[0];
                    actualDelta = ttTraj_safe[0];
                    actualYaw = yawFromOdom;
                }
                
                break;

            case 2:
                if(traj_nominal.size() > 0){
                    startTime = ros::Time::now();
                    execTraj = 0;
                }

                break;
            }

            if(execTraj != 2){
                geometry_msgs::Point p;
                Eigen::Matrix<double, 6, 1> B;

                tt = tt/actualDelta;
                B << pow(1-tt, 5), 5*tt*pow(1-tt,4), 10*pow(tt,2)*pow(1-tt,3), 10*pow(tt,3)*pow(1-tt,2), 5*pow(tt,4)*(1-tt), pow(tt,5);
                lastSP = actualTraj*B;
                lastYawSP = actualYaw*M_PI/180.0f;

                p.x = lastSP(0);
                p.y = lastSP(1);
                p.z = lastSP(2);
                
                targetMsg.position.x = lastSP(0);
                targetMsg.position.y = lastSP(1);
                targetMsg.position.z = lastSP(2);
                targetMsg.yaw = lastYawSP;
            } else{
                targetMsg.position.x = lastSP(0);
                targetMsg.position.y = lastSP(1);
                targetMsg.position.z = lastSP(2);
                targetMsg.yaw = lastYawSP;
            }
        
            break;
        }

        targetMsg.type_mask =   mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_VX  |
                                mavros_msgs::PositionTarget::IGNORE_VY  |
                                mavros_msgs::PositionTarget::IGNORE_VZ  |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        targetMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        targetMsg.header.stamp = ros::Time::now();
        target_pub.publish(targetMsg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}