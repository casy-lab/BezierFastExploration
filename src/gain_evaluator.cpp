/* BSpline Exploration                         */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: February 2022                         */
/* File: gain_evaluator.cpp                    */

// External Libraries
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

// STD Library
#include <iostream>
#include <queue>
#include <tuple>

// Standard MSGS
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Odometry.h>

// Custom
#include <bezier_exploration/TreeNode.h>

class GainEvaluator{
  public:
  // Class Constructor
  GainEvaluator(ros::NodeHandle& nh);

  // Class Destructor
  ~GainEvaluator(){;}

  private:
  // Attributes
  int horizontalFOV = 0,
      verticalFOV   = 0,
      deltaTheta    = 0;
  
  double  deltaRadius    = 0.0,
          minRadius      = 0.0,
          maxRadius      = 0.0,
          deltaPhi_rad   = 0.0,
          deltaTheta_rad = 0.0,
          deltaPhi       = 0.0;

  double  mapBoundMaxX    = 0.0,
          mapBoundMinX    = 0.0,
          mapBoundMaxY    = 0.0,
          mapBoundMinY    = 0.0,
          mapBoundMaxZ    = 0.0,
          mapBoundMinZ    = 0.0;

  // Structures
  std::shared_ptr<octomap::OcTree> ot_;
  std::queue<std::tuple<Eigen::Matrix<double, 3, 1>, int>> nodeQueue;

  // Subscribers
  ros::Subscriber gainEvaluation, octomapSubscriber;

  // Publishers
  ros::Publisher nodePublisher;

  // Timers
  ros::Timer executionTimer;

  // Helper Functions
  void computeExplicitGain(Eigen::Matrix<double, 3, 1> pose, int id);
  bool insideBounds(double px, double py, double pz);

  // Callback Functions
  void execute(const ros::TimerEvent& e);
  void gainCallback(const bezier_exploration::TreeNode::ConstPtr& msg);
  void octomapCallback(const octomap_msgs::Octomap& msg);
};

GainEvaluator::GainEvaluator(ros::NodeHandle& nh):
  gainEvaluation(nh.subscribe("/gain_eval", 1, &GainEvaluator::gainCallback, this)),
  octomapSubscriber(nh.subscribe("/octomap_full", 1, &GainEvaluator::octomapCallback, this)),
  nodePublisher(nh.advertise<bezier_exploration::TreeNode>("/node_tree_gain", 1)),
  executionTimer(nh.createTimer(ros::Duration(0.025), &GainEvaluator::execute, this)){

  nh.param("rrt/horizontal_fov", horizontalFOV, 115);
  nh.param("rrt/vertical_fov", verticalFOV, 60);
  nh.param("rrt/delta_r", deltaRadius, 0.1);
  nh.param("rrt/min_r", minRadius, 0.3);
  nh.param("rrt/max_r", maxRadius, 6.0);
  nh.param("rrt/delta_phi", deltaPhi, 10.0);
  nh.param("rrt/delta_theta", deltaTheta, 10);

  nh.param("map/x_max", mapBoundMaxX, 34.0);
  nh.param("map/x_min", mapBoundMinX, -4.0);
  nh.param("map/y_max", mapBoundMaxY, 6.0);
  nh.param("map/y_min", mapBoundMinY, -24.0);
  nh.param("map/z_max", mapBoundMaxZ, 24.0);
  nh.param("map/z_min", mapBoundMinZ, 2.0);

  deltaPhi_rad = (deltaPhi * M_PI)/180.0f;
  deltaTheta_rad = ((double)deltaTheta * M_PI)/180.0f;
}

void GainEvaluator::octomapCallback(const octomap_msgs::Octomap& msg){
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
}

void GainEvaluator::gainCallback(const bezier_exploration::TreeNode::ConstPtr& msg){
  Eigen::Matrix<double, 3, 1> node;
  int id = msg->id;

  node(0) = msg->x;
  node(1) = msg->y;
  node(2) = msg->z;

  nodeQueue.push(std::make_tuple(node, id));
}

void GainEvaluator::execute(const ros::TimerEvent& e){
  if(!ot_){
    ROS_WARN("No Octomap Received!");
    return;
  }

  if(nodeQueue.size() > 0){
    std::tuple<Eigen::Matrix<double, 3, 1>, int> actualTuple = nodeQueue.front();
    nodeQueue.pop();

    // Unpack Tuple
    Eigen::Matrix<double, 3, 1> actualNode = std::get<0>(actualTuple);
    int id = std::get<1>(actualTuple);

    // Execute
    computeExplicitGain(actualNode, id);
  }
}

void GainEvaluator::computeExplicitGain(Eigen::Matrix<double, 3, 1> pose, int id){
  std::shared_ptr<octomap::OcTree> ot = ot_;                                  

  double theta_rad, phi_rad;
  std::map<int, double> yawGain;
  for(int theta = 0; theta < 360; theta += deltaTheta){
    theta_rad = ((double)theta*M_PI)/180.0f;
    for(double phi = (90.0f - (verticalFOV/2.0f)); phi < (90.0f + (verticalFOV/2.0f)); phi += deltaPhi){
      phi_rad = (phi*M_PI)/180.0f;

      double temp_gain = 0.0;
      for(double R = minRadius; R < maxRadius; R += deltaRadius){
        double rho = (2*R*R*deltaRadius + (deltaRadius*deltaRadius*deltaRadius)/6)*deltaTheta_rad*sin(phi_rad)*sin(deltaPhi_rad/2);
        double px = pose(0) + R*cos(theta_rad)*sin(phi_rad);
        double py = pose(1) + R*sin(theta_rad)*sin(phi_rad);
        double pz = pose(2) + R*cos(phi_rad);

        if(!insideBounds(px, py, pz)){
          temp_gain -= rho*(maxRadius-R);
          break;
        }
        octomap::point3d query(px, py, pz);
        octomap::OcTreeNode* result = ot->search(query);

        if(result){
          if(result->getOccupancy() > 0.5){
            break;
          }
        } else{
          temp_gain += rho;
        }
      }

      yawGain[theta] += temp_gain;
    }
  }

  int bestYaw = 0;
  double bestYawGain = -INFINITY;
  for(int yawAngle = 0; yawAngle < 360; yawAngle += deltaTheta){
    double yawTotalGain = 0.0;
    int hFOV_2 = round(horizontalFOV/2.0);
    for(int fov = -hFOV_2; fov < hFOV_2; fov++){
      int theta = yawAngle + fov;

      if(theta%deltaTheta != 0){
        continue;
      }

      if(theta < 0)
        theta += 360;
      if(theta > 360)
        theta -= 360;

      yawTotalGain += yawGain[theta];
    }

    if(bestYawGain < yawTotalGain){
      bestYawGain = yawTotalGain;
      bestYaw = yawAngle;
    }
  }

  // Publish Node
  bezier_exploration::TreeNode treeNode;

  treeNode.x = pose(0);
  treeNode.y = pose(1);
  treeNode.z = pose(2);
  treeNode.yaw = (double)bestYaw;
  treeNode.gain = bestYawGain;
  treeNode.id = id;

  nodePublisher.publish(treeNode);
}

bool GainEvaluator::insideBounds(double px, double py, double pz){
  if(px > mapBoundMaxX || px < mapBoundMinX)
    return false;

  if(py > mapBoundMaxY || py < mapBoundMinY)
    return false;

  if(pz > mapBoundMaxZ || pz < mapBoundMinZ)
    return false;

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "gain_evaluator_node");
  ros::NodeHandle nh("~");

  GainEvaluator evaluator(nh);
  ros::spin();

  return 0;
}