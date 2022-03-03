/* Bezier Exploration                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: February 2022                         */
/* File: bezier_exploration.hpp                */

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
#include <random>
#include <ctime>
#include <cstdlib>

// Standard MSGS
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>

// Custom
#include <bezier_exploration/InitService.h>
#include <bezier_exploration/BezierTraj.h>
#include <bezier_exploration/gpMsgs.h>

template <typename T>
T getBMatrix(double deltaTime){
    T B;
    switch(B.rows()){
        case 4:{       
            B << 0.1429, 0.0714, 0.0286, 0.0071,
                 0.0714, 0.0857, 0.0643, 0.0286,
                 0.0286, 0.0643, 0.0857, 0.0714,
                 0.0071, 0.0286, 0.0714, 0.1429;
            break;
        }
        case 3:{
            B <<  0.2,  0.1,  0.03,
                  0.1,  0.13, 0.1,
                  0.03, 0.1,  0.2;
            break;
        }
    }
    B *= deltaTime;
    return B;
}

template <typename T>
T getQVMatrix(double deltaTime){
    T QV;
    switch(QV.rows()){
        case 5:{
            QV << -1,  1,  0,  0,  0,  0,
                   0, -1,  1,  0,  0,  0,
                   0,  0, -1,  1,  0,  0,
                   0,  0,  0, -1,  1,  0,
                   0,  0,  0,  0, -1,  1;
            QV *= 5/deltaTime;
            break;
        }
        case 3:{
            QV << -1,  1,  0,  0,
                   0, -1,  1,  0,
                   0,  0, -1,  1;
            QV *= 3/deltaTime;
            break;
        }
    }
    return QV;
}

template <typename T>
T getQAMatrix(double deltaTime){
    T QA;
    switch(QA.rows()){
        case 4:{
            QA << -1,  1,  0,  0, 0,
                   0, -1,  1,  0, 0,
                   0,  0, -1,  1, 0,
                   0,  0,  0, -1, 1;
            QA *= 4/deltaTime;
            break;
        }
    }
    return QA;
}

template <typename T>
T getCMatrix(){
    T C;
    switch(C.rows()){
        case 6:{
            C <<  0, 0, 1, 0, 0, 0,
                  0, 0, 0, 1, 0, 0,
                  0, 0, 0, 0, 1, 0,
                  1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 1;
            break;
        }
        case 4:{
            C << 0, 1, 0, 0,
                 0, 0, 1, 0,
                 1, 0, 0, 0,
                 0, 0, 0, 1;
            break;
        }
    }
  return C;
}

typedef enum{TRAJECTORY_FOUND,
             NO_NEW_NODE} expansionResult;
             
typedef enum{EXPLORATION_ENDED,
             SAFE_TRAJECTORY_NOT_FOUND,
             DEADLINE_MISSED,
             NULL_IDX} explorationState;

class Node{
    private:
    bool branchActive = true;
    bool isRoot = false;

    public:
    // Node Attributes
    double  informationGain    = 0.0,
            executionTime      = 0.0,
            trajectoryCost     = 0.0;

    std::vector<Eigen::Matrix<double, 3, 1>> controlPoints_pose;
    std::vector<double> controlPoints_yaw;

    Node* nodeParent = this;
    std::vector<Node*> nodeChilds;

    // Class Constructor
    Node(){
        controlPoints_pose.reserve(6);
        controlPoints_yaw.reserve(4);
    }

    // Class Destructor
    ~Node(){;}

    // Helper Functions
    inline bool isNodeRoot(){
        return isRoot;
    }

    inline bool isNodeActive(){
        return branchActive;
    }

    void setAsRootNode(std::vector<Node*>& nodeTree){
        for(std::vector<Node*>::iterator iter = nodeTree.begin();
            iter != nodeTree.end();
            iter++){

            (*iter)->setRootIdx(false);
        }
        
        setRootIdx(true);
    }

    inline void setRootIdx(bool set){
        isRoot = set;
    }

    inline void setNodeActivation(bool set){
        branchActive = set;
    }

    void setBranchActivation(bool set){
        setNodeActivation(set);

        if(nodeChilds.size() > 0){
            for(std::vector<Node*>::iterator childIter = nodeChilds.begin();
                childIter != nodeChilds.end();
                childIter++){
                
                (*childIter)->setBranchActivation(set);
            }
        }
    }

    double getTotalUtility(){
        if(trajectoryCost <= 0.0){
            return 0.0;
        }

        return getBranchGain()/getBranchCost();
    }

    double getBranchCost(){
        if(nodeParent == this || !nodeParent->isNodeActive()){
            return trajectoryCost;
        } else{
            return nodeParent->getBranchCost() + trajectoryCost;
        }
    }

    double getBranchGain(){
        if(nodeParent == this || !nodeParent->isNodeActive()){
            return informationGain;
        } else{
            return nodeParent->getBranchGain() + informationGain;
        }
    }
};

class BezierExplorer{
    private:
    // RRT Constants
    double  safeDistance    = 0.0,
            maxSamplingRad  = 0.0,
            minSamplingRad  = 0.0,
            minimumDelta    = 0.0,
            maximumDelta    = 0.0,
            timeResolution  = 0.0,
            rho_time        = 0.0,
            rho_acc         = 0.0,
            rho_wz          = 0.0;

    int     maxSampledNodes = 0,
            maxIterNum      = 0,
            trajNumber      = 0,
            maxTrajNumber   = 0;

    // Map Constants
    double  mapBoundMaxX    = 0.0,
            mapBoundMinX    = 0.0,
            mapBoundMaxY    = 0.0,
            mapBoundMinY    = 0.0,
            mapBoundMaxZ    = 0.0,
            mapBoundMinZ    = 0.0;

    // Dynamics Constraints
    double  maxVelocity     = 0.0,
            maxAcceleration = 0.0;

    // Flags
    bool verbose = true,
         verbose_debug = false,
         verbose_vis = true,
         initializationDone = false,
         readyForSending = true;

    // Memory
    ros::Time previousIter;
    ros::Duration iterTime;
    double  previousDelta       = 0.0,
            previousSafeDelta   = 0.0;

    // Structures
    std::shared_ptr<octomap::OcTree> ot_;
    std::vector<double> times;
    std::vector<Node*> nodeTree;
    std::vector<float> alphaMatrix_gp, hyperParams_gp; // Row-Major Format
    std::vector<float> xData_gp, yData_gp, zData_gp, yawData_gp;
    Node* fatherNode = NULL;
    Node* bestNode = NULL;
    Node* bestBranchNode = NULL;
    Node* bestBranchNode_prev = NULL;
    Node lastSentSafe;
    visualization_msgs::Marker markerTraj;
    explorationState actualExplorationState = NULL_IDX;

    // Subscribers
    ros::Subscriber octomapSubscriber, executedSub, gpTrainingSub;

    // Publishers
    ros::Publisher  trajPublisher,
                    queryPublisher,
                    trajVisPublisher,
                    treePublisher,
                    branchPublisher;

    // Services
    ros::ServiceServer initService;

    // Timers
    ros::Timer executeTimer, publishingTimer;

    // Main Function
    void explore();

    // Helper Functions
    void findBestDelta(Node* parentNode, std::vector<Eigen::Matrix<double, 3, 1>>& newControlPoints_pose,
                       std::vector<double>& newControlPoints_yaw, Eigen::Matrix<double, 3, 1> newPosition,
                       double optimalYaw, double& deltaTime, double& optimalCost, bool finalPoint, bool trajReversed);
    void findBestDelta_rewire(Node* parentNode, Node* childNode, std::vector<Eigen::Matrix<double, 3, 1>>& controlPoints_pose,
                              std::vector<double>& controlPoints_yaw, double& deltaTime, double& optimalCost);
    void planSafeTrajectory(Node* node, std::vector<Eigen::Matrix<double, 3, 1>>& controlPoints_pose,
                            std::vector<double>& controlPoints_yaw, double& deltaTime, double& optimalCost, bool trajReversed);
    expansionResult growRRT();
    void rewireSafeTrajectory();
    bool collisionPoint(octomap::point3d query, double distance);
    void initializeTree(Eigen::Matrix<double, 3, 1> actualPosition, double actualYaw);
    bool insideBounds(Eigen::Matrix<double, 3, 1> position);
    void samplePoint(Eigen::Matrix<double, 3, 1>& newPoint, Eigen::Matrix<double, 3, 1> prevPoint, bool useMinRadius);
    void deleteCildrenNodes(Node* node);
    void maskUnfeasibleBranches(Node* node, Node* nodePrev);
    std::vector<float> kernel(Eigen::Matrix<double, 3, 1> point);
    double getGain(Eigen::Matrix<double, 3, 1> pose, double& yaw);
    Node* findFather(Eigen::Matrix<double, 3, 1> newPoint);
    void publishTree();
    void publishTraj(bezier_exploration::BezierTraj msg);

    // Callback Functions
    void execute(const ros::TimerEvent& e);
    void publishBestBranch(const ros::TimerEvent& e);
    void executedCallback(const std_msgs::Empty::ConstPtr& msg){trajNumber--;};
    void gpTrainingCallback(const bezier_exploration::gpMsgs::ConstPtr& msg){
        alphaMatrix_gp.clear();
        alphaMatrix_gp.resize(msg->alpha.size());
        alphaMatrix_gp = msg->alpha;

        hyperParams_gp.clear();
        hyperParams_gp.resize(msg->params.size());
        hyperParams_gp = msg->params;

        xData_gp.clear();
        xData_gp.resize(msg->xData.size());
        xData_gp = msg->xData;

        yData_gp.clear();
        yData_gp.resize(msg->yData.size());
        yData_gp = msg->yData;

        zData_gp.clear();
        zData_gp.resize(msg->zData.size());
        zData_gp = msg->zData;

        yawData_gp.clear();
        yawData_gp.resize(msg->yawData.size());
        yawData_gp = msg->yawData;
    };
    bool setInit(bezier_exploration::InitService::Request &req,
                 bezier_exploration::InitService::Response &res){

        Eigen::Matrix<double, 3, 1> actualPosition(req.x, req.y, req.z);
        
        geometry_msgs::Point queryGain;
        queryGain.x = actualPosition(0);
        queryGain.y = actualPosition(1);
        queryGain.z = actualPosition(2);
        queryPublisher.publish(queryGain);

        double actualYaw = req.yaw;
        initializeTree(actualPosition, actualYaw);
        lastSentSafe.controlPoints_pose.insert(lastSentSafe.controlPoints_pose.begin(), 6, actualPosition);
        lastSentSafe.controlPoints_yaw.insert(lastSentSafe.controlPoints_yaw.begin(), 4, actualYaw);
        initializationDone = true;
        return true;
    };
    void octomapCallback(const octomap_msgs::Octomap& msg){
        octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
        octomap::OcTree* ot = (octomap::OcTree*)aot;
        ot_ = std::make_shared<octomap::OcTree>(*ot);

        delete ot;
    };

    public:
    // Class Constructor
    BezierExplorer(ros::NodeHandle& nh);

    // Class Destructor
    ~BezierExplorer();
};