/* Bezier Exploration                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: February 2022                         */
/* File: bezier_exploration.cpp                */

#include <bezier_exploration/bezier_exploration.hpp>

// Class Constructor
BezierExplorer::BezierExplorer(ros::NodeHandle& nh): 
  octomapSubscriber(nh.subscribe("/octomap_full", 1, &BezierExplorer::octomapCallback, this)),
  executedSub(nh.subscribe("/traj_executed", 10, &BezierExplorer::executedCallback, this)),
  gpTrainingSub(nh.subscribe("/gp_params", 1, &BezierExplorer::gpTrainingCallback, this)),
  trajPublisher(nh.advertise<bezier_exploration::BezierTraj>("/traj", 1)),
  queryPublisher(nh.advertise<geometry_msgs::Point>("/query_node", 1)),
  trajVisPublisher(nh.advertise<visualization_msgs::Marker>("/traj_vis", 1)),
  treePublisher(nh.advertise<visualization_msgs::Marker>("/tree_node", 1)),
  branchPublisher(nh.advertise<visualization_msgs::Marker>("/tree_branch", 1)),
  initService(nh.advertiseService("/init", &BezierExplorer::setInit, this)),
  executeTimer(nh.createTimer(ros::Duration(0.1), &BezierExplorer::execute, this)),
  publishingTimer(nh.createTimer(ros::Duration(0.02), &BezierExplorer::publishBestBranch, this)){

  nh.param("rrt/safe_distance", safeDistance, 0.5);
  nh.param("rrt/max_radius", maxSamplingRad, 3.0);
  nh.param("rrt/min_radius", minSamplingRad, 1.0);
  nh.param("rrt/tree_max_dim", maxSampledNodes, 20);
  nh.param("rrt/max_iterations", maxIterNum, 50);
  nh.param("rrt/minimum_delta", minimumDelta, 1.0);
  nh.param("rrt/maximum_delta", maximumDelta, 5.0);
  nh.param("rrt/time_resolution", timeResolution, 0.5);
  nh.param("rrt/weight_time", rho_time, 0.5);
  nh.param("rrt/weight_acc", rho_acc, 0.1);
  nh.param("rrt/weight_yaw", rho_wz, 0.1);
  nh.param("rrt/max_traj_number", maxTrajNumber, 2);
  nh.param("rrt/verbose", verbose, true);
  nh.param("rrt/verbose_debug", verbose_debug, false);
  nh.param("rrt/verbose_vis", verbose_vis, true);
  nh.param("uav/max_velocity", maxVelocity, 1.5);
  nh.param("uav/max_acceleration", maxAcceleration, 1.5);
  nh.param("map/x_max", mapBoundMaxX, 35.0);
  nh.param("map/x_min", mapBoundMinX, -2.0);
  nh.param("map/y_max", mapBoundMaxY, 8.0);
  nh.param("map/y_min", mapBoundMinY, -25.0);
  nh.param("map/z_max", mapBoundMaxZ, 26.0);
  nh.param("map/z_min", mapBoundMinZ, 1.0);

  for(double delta = minimumDelta; delta < maximumDelta; delta += timeResolution){
    times.push_back(delta);
  }

  markerTraj.id = 0;
  std::srand(std::time(nullptr));
}

// Class Destructor
BezierExplorer::~BezierExplorer(){
  for(uint i = 0; i < nodeTree.size(); i++)
    delete nodeTree[i];
}

// Timer -> Publish Best Branch
void BezierExplorer::publishBestBranch(const ros::TimerEvent& e){
  if(trajNumber >= maxTrajNumber || nodeTree.size() <= 1 || actualExplorationState != NULL_IDX || !readyForSending){
    return;
  }

  if(bestBranchNode->isNodeRoot()){
    return;
  }

  ROS_INFO_COND(verbose, "Publishing New Trajectory, Tree Size %ld", nodeTree.size());

  // Non-NULL Tree - Publish Best Branch
  // Plan Safe-End Trajectory
  double safeTime = 0.0, safeCost = 0.0;
  std::vector<Eigen::Matrix<double, 3, 1>> safePoints;
  std::vector<double> safePoints_yaw;
  int count = 0;
  do{
    planSafeTrajectory(bestBranchNode, safePoints, safePoints_yaw, safeTime, safeCost, false);

    iterTime = ros::Time::now() - previousIter;
    if(iterTime.toSec() > previousDelta){
      ROS_WARN_COND(verbose, "Deadline Missed During Safe Planning");
      actualExplorationState = DEADLINE_MISSED;
      return;
    }

    if(count > maxIterNum*3 && previousDelta == INFINITY){
      actualExplorationState = SAFE_TRAJECTORY_NOT_FOUND;
      return;
    } else{
      count ++;
    }
    
  } while(safeTime < 0);

  // Safe Trajectory FOUND! Continue!
  // Memorize Safe
  lastSentSafe.informationGain = 0.0;
  lastSentSafe.controlPoints_yaw = safePoints_yaw;
  lastSentSafe.executionTime = safeTime;
  lastSentSafe.trajectoryCost = safeCost;
  lastSentSafe.controlPoints_pose = safePoints;
  lastSentSafe.nodeParent = bestBranchNode;

  // Compute Remaining Time
  iterTime = ros::Time::now() - previousIter;
  double remainingFromPrevious = (previousDelta == INFINITY) ? 0.0 : previousDelta - iterTime.toSec();

  previousDelta = (remainingFromPrevious > 0) ? bestBranchNode->executionTime + remainingFromPrevious : bestBranchNode->executionTime;
  previousSafeDelta = previousDelta + safeTime;

  // Send Computed Trajectories
  // Nominal Trajectory
  bezier_exploration::BezierTraj msg;
  msg.time = bestBranchNode->executionTime;
  msg.time_safe = safeTime;

  for(uint ii = 0; ii < bestBranchNode->controlPoints_pose.size(); ii++){
    Eigen::Matrix<double, 3, 1> actualPoint = bestBranchNode->controlPoints_pose[ii];
    msg.x.push_back(actualPoint(0));
    msg.y.push_back(actualPoint(1));
    msg.z.push_back(actualPoint(2));
  }

  for(uint ii = 0; ii < bestBranchNode->controlPoints_yaw.size(); ii++){
    msg.yaw.push_back(bestBranchNode->controlPoints_yaw[ii]);
  }

  // Safe Trajectory
  for(uint ii = 0; ii < safePoints.size(); ii++){
    Eigen::Matrix<double, 3, 1> actualPoint = safePoints[ii];
    msg.x_safe.push_back(actualPoint(0));
    msg.y_safe.push_back(actualPoint(1));
    msg.z_safe.push_back(actualPoint(2));
  }

  trajPublisher.publish(msg);

  // Set Previous Iter Time
  previousIter = ros::Time::now();
  trajNumber++;

  // Refresh Root Node
  bestBranchNode->setAsRootNode(nodeTree);

  // Mask Unfeasible Branches
  maskUnfeasibleBranches(bestBranchNode, bestBranchNode_prev);
  bestBranchNode_prev = bestBranchNode;

  // For Visualization Purpose
  if(verbose_vis){
    publishTree();
    publishTraj(msg);
  }

  ROS_INFO_COND(verbose, "Trajectory Published");

  // Re-Set Father Node
  double actualUtility = -INFINITY;
  for(std::vector<Node*>::iterator treeIter = nodeTree.begin();
      treeIter != nodeTree.end();
      treeIter++){

    double optimalYaw;
    double nodeGain = getGain((*treeIter)->controlPoints_pose[(*treeIter)->controlPoints_pose.size()-1], optimalYaw);
    (*treeIter)->controlPoints_yaw[3] = optimalYaw;
    (*treeIter)->informationGain = nodeGain;

    if(!(*treeIter)->isNodeActive()){
      continue;
    }

    if((*treeIter)->getTotalUtility() > actualUtility){
      fatherNode = *treeIter;
      actualUtility = fatherNode->getTotalUtility();
    }
  }
}

void BezierExplorer::maskUnfeasibleBranches(Node* node, Node* nodePrev){
  ROS_INFO_COND(true, "Performimg Mask");
  if(nodePrev == NULL || nodePrev == node->nodeParent){
    Node* previousNode = node->nodeParent;
    if(previousNode->nodeChilds.size() != 0){
      for(std::vector<Node*>::iterator childIter = previousNode->nodeChilds.begin();
          childIter != previousNode->nodeChilds.end();
          childIter++){
        
        if(*childIter != node){
          (*childIter)->setBranchActivation(false);
        }
      }
    }

    previousNode->setNodeActivation(false);
  }
}

// Timer -> Main Loop
void BezierExplorer::execute(const ros::TimerEvent& e){
  if(!initializationDone || !ot_ || alphaMatrix_gp.size() == 0 || trajNumber >= maxTrajNumber){
    return;
  }

  switch(actualExplorationState){
    case DEADLINE_MISSED:{
      // Deadline Missed
      // Wait for Safe Traj. Execution
      readyForSending = false;
      iterTime = ros::Time::now() - previousIter;

      if(iterTime.toSec() > previousSafeDelta){
        ROS_INFO_COND(verbose, "Resetting Deadline");

        rewireSafeTrajectory();
        actualExplorationState = NULL_IDX;
      }
      break;
    }

    case SAFE_TRAJECTORY_NOT_FOUND:{
      ROS_WARN_COND(verbose, "Safe Trajectory Not Found!!");
      initializeTree(lastSentSafe.controlPoints_pose[lastSentSafe.controlPoints_pose.size()-1],
                     lastSentSafe.controlPoints_yaw[lastSentSafe.controlPoints_yaw.size()-1]);

      actualExplorationState = NULL_IDX;
      break;
    }

    case EXPLORATION_ENDED:{
      ROS_WARN_COND(verbose, "Exploration Ended");
      // Absorbing State
      break;
    }

    case NULL_IDX:{
      // Perform an Exploration Step
      explore();
      readyForSending = true;
      break;
    }
  }
}

void BezierExplorer::initializeTree(Eigen::Matrix<double, 3, 1> actualPosition, double actualYaw){
  // This is the First Initialization
  ROS_INFO_COND(verbose, "Tree Initialization");

  Node* newNode = new Node();
  newNode->controlPoints_pose.insert(newNode->controlPoints_pose.begin(), 6, actualPosition);
  newNode->controlPoints_yaw.insert(newNode->controlPoints_yaw.begin(), 4, actualYaw);
  newNode->executionTime = 1.0; // To Avoid T/0

  previousDelta = INFINITY;
  previousSafeDelta = INFINITY;

  // Ensure The Tree is Empty
  if(nodeTree.size() != 0){
    for(std::vector<Node*>::iterator iter = nodeTree.begin();
        iter != nodeTree.end(); iter ++){

      delete *iter;
    }

    nodeTree.clear();
    nodeTree.shrink_to_fit();
  }

  // Push Back the First Node -> Actual Position!
  nodeTree.push_back(newNode);
  newNode->setAsRootNode(nodeTree);

  fatherNode = newNode;
  bestBranchNode_prev = NULL;
}

void BezierExplorer::rewireSafeTrajectory(){
  // Add Safe Trajectory as Node in The Tree
  Node* safeNode = new Node();
  safeNode->controlPoints_yaw = lastSentSafe.controlPoints_yaw;
  safeNode->controlPoints_pose = lastSentSafe.controlPoints_pose;
  safeNode->executionTime = lastSentSafe.executionTime;
  safeNode->trajectoryCost = lastSentSafe.trajectoryCost;
  safeNode->nodeParent = lastSentSafe.nodeParent;

  // Try To Rewire safeNode WITH lastSentSafe.nodeParent->nodeChilds
  if(lastSentSafe.nodeParent->nodeChilds.size() != 0){
    for(std::vector<Node*>::iterator childIter = lastSentSafe.nodeParent->nodeChilds.begin();
        childIter != lastSentSafe.nodeParent->nodeChilds.end();
        childIter++){

      std::vector<Eigen::Matrix<double, 3, 1>> controlPoints_pose;
      std::vector<double> controlPoints_yaw;
      double deltaTime = 0.0, optimalCost = 0.0;

      findBestDelta_rewire(safeNode, *childIter, controlPoints_pose, controlPoints_yaw, deltaTime, optimalCost);

      if(deltaTime <= 0.0){
        // Rewire not Possible!
        deleteCildrenNodes(*childIter);
        continue;
      }

      Node* rewiringNode = new Node();
      rewiringNode->informationGain = 0.0;
      rewiringNode->controlPoints_pose = controlPoints_pose;
      rewiringNode->controlPoints_yaw = controlPoints_yaw;
      rewiringNode->executionTime = deltaTime;
      rewiringNode->trajectoryCost = optimalCost;
      rewiringNode->nodeParent = safeNode;
      
      safeNode->nodeChilds.push_back(rewiringNode);
      rewiringNode->nodeChilds.push_back(*childIter);
      (*childIter)->nodeParent = rewiringNode;
      nodeTree.push_back(rewiringNode);
    }
  }

  lastSentSafe.nodeParent->nodeChilds.clear();
  lastSentSafe.nodeParent->nodeChilds.shrink_to_fit();
  lastSentSafe.nodeParent->nodeChilds.push_back(safeNode);
  safeNode->setAsRootNode(nodeTree);
  nodeTree.push_back(safeNode);

  bestBranchNode = safeNode;
  bestBranchNode_prev = safeNode;

  double actualGain = -INFINITY;
  fatherNode = safeNode;
  for(std::vector<Node*>::iterator treeIter = nodeTree.begin();
      treeIter != nodeTree.end();
      treeIter++){

    double optimalYaw;
    double nodeGain = getGain((*treeIter)->controlPoints_pose[(*treeIter)->controlPoints_pose.size()-1], optimalYaw);
    (*treeIter)->controlPoints_yaw[3] = optimalYaw;
    (*treeIter)->informationGain = nodeGain;

    if(!(*treeIter)->isNodeActive()){
      continue;
    }

    if((*treeIter)->informationGain > actualGain){
      fatherNode = *treeIter;
      actualGain = fatherNode->informationGain;
    }
  }

  previousDelta = INFINITY;
}

void BezierExplorer::deleteCildrenNodes(Node* node){
  if(node->nodeChilds.size() != 0){
    for(std::vector<Node*>::iterator childIter = node->nodeChilds.begin();
        childIter != node->nodeChilds.end();
        childIter ++){

      deleteCildrenNodes(*childIter);
    }
  }

  for(std::vector<Node*>::iterator iter = nodeTree.begin();
      iter != nodeTree.end();
      iter ++){
    
    if(*iter == node){
      delete *iter;
      nodeTree.erase(iter);
      break;
    }
  }
}

// Run Bezier Exploration
void BezierExplorer::explore(){
  ROS_INFO_COND(verbose, "Start Exploration Step");
  ros::Time startLoop_time = ros::Time::now();

  expansionResult actualResult = NO_NEW_NODE;
  do{
    ROS_INFO_COND(verbose, "Expand Tree");
    ros::Time singleCicle_time = ros::Time::now();

    actualResult = growRRT();

    if((ros::Time::now() - previousIter).toSec() >= previousDelta){
      ROS_WARN_COND(verbose, "Deadline Missed!");
      actualExplorationState = DEADLINE_MISSED;
      return;
    }

    if(!bestNode->isNodeRoot()){
      actualResult = TRAJECTORY_FOUND;
    }

    ROS_INFO_COND(verbose, "Cicle: %f", (ros::Time::now() - singleCicle_time).toSec());
  } while(actualResult != TRAJECTORY_FOUND);

  ROS_INFO_COND(verbose, "End Expansion, Elapsed Time: %f", (ros::Time::now() - startLoop_time).toSec());

  // Extract Best Branch
  Node* actualBranch = bestNode;
  Node* lastParent = bestNode;
  bestBranchNode = bestNode;
  bool childSeries = false;
  while(!actualBranch->isNodeRoot()){
    if(actualBranch->nodeParent->isNodeActive() && !childSeries){
      bestBranchNode = actualBranch;
      actualBranch = actualBranch->nodeParent;
      lastParent = actualBranch;

    } else{
      childSeries = true;
      for(std::vector<Node*>::iterator childIter = actualBranch->nodeChilds.begin();
          childIter != actualBranch->nodeChilds.end();
          childIter++){

        if((*childIter) == lastParent){
          continue;
        }
        
        if((*childIter)->isNodeActive()){
          lastParent = actualBranch;
          bestBranchNode = actualBranch;
          actualBranch = *childIter;

          break;
        }
      }
    }
  }

  ROS_INFO_COND(verbose, "Best Node Extracted");

  // Re-Set Father Node
  double actualUtility = -INFINITY;
  for(std::vector<Node*>::iterator treeIter = nodeTree.begin();
      treeIter != nodeTree.end();
      treeIter++){

    double optimalYaw;
    double nodeGain = getGain((*treeIter)->controlPoints_pose[(*treeIter)->controlPoints_pose.size()-1], optimalYaw);
    (*treeIter)->controlPoints_yaw[3] = optimalYaw;
    (*treeIter)->informationGain = nodeGain;

    if(!(*treeIter)->isNodeActive()){
      continue;
    }

    if((*treeIter)->getTotalUtility() > actualUtility){
      fatherNode = *treeIter;
      actualUtility = fatherNode->getTotalUtility();
    }  
  }
}

expansionResult BezierExplorer::growRRT(){
  std::shared_ptr<octomap::OcTree> ot = ot_;
  bestNode = fatherNode;

  if(bestNode->isNodeRoot()){
    bestNode->informationGain = 0.0;
  }

  double bestUtility = bestNode->getTotalUtility();

  for(int nodeNumber = 0;
     (nodeNumber < maxSampledNodes) && (ros::ok());
      nodeNumber++){

    // Sample New Point
    Eigen::Matrix<double, 3, 1> newPosition;
    octomap::OcTreeNode* result;
    octomap::point3d query;

    do{
      samplePoint(newPosition, bestNode->controlPoints_pose[bestNode->controlPoints_pose.size()-1], true);

      query = octomap::point3d(newPosition(0), newPosition(1), newPosition(2));
      result = ot_->search(query);

      if(!result){
        ROS_INFO_COND(verbose_debug, "Not Observed");
      }
    } while(!result || collisionPoint(query, safeDistance));

    // Found Father
    Node* actualFatherNode = findFather(newPosition);

    // Compute Information Gain
    double optimalYaw;
    double nodeGain = getGain(newPosition, optimalYaw);

    // Check For Feasible/Best Execution Time
    double executionTime = -1.0, optimalCost = 0.0;
    std::vector<Eigen::Matrix<double, 3, 1>> controlPoints_pose_new;
    std::vector<double> controlPoints_yaw_new;
    findBestDelta(actualFatherNode, controlPoints_pose_new, controlPoints_yaw_new, newPosition, optimalYaw, executionTime, optimalCost, false, false);

    // No Fesible Delta -> Abort
    if(executionTime < 0){
      ROS_INFO_COND(verbose_debug, "No Delta");
      continue;
    }else{
      ROS_INFO_COND(verbose_debug, "Delta Found");
    }

    // All Checks Passed
    // Add Node to Tree
    Node* newNode = new Node();
    newNode->executionTime = executionTime;
    newNode->nodeParent = actualFatherNode;
    newNode->controlPoints_pose = controlPoints_pose_new;
    newNode->controlPoints_yaw = controlPoints_yaw_new;
    newNode->trajectoryCost = optimalCost;
    newNode->informationGain = nodeGain;

    // Update Best Node
    ROS_INFO_COND(verbose, "Best Node Utility %f", bestNode->getTotalUtility());
    ROS_INFO_COND(verbose, "Best Node Gain %f", bestNode->informationGain);
    ROS_INFO_COND(verbose, "New Node Utility %f", newNode->getTotalUtility());
    ROS_INFO_COND(verbose, "New Node Gain %f", newNode->informationGain);
    
    actualFatherNode->nodeChilds.push_back(newNode);
    nodeTree.push_back(newNode);

    double actualUtility = newNode->getTotalUtility();
    if(actualUtility > bestUtility){
      bestNode = newNode;
      bestUtility = bestNode->getTotalUtility();
    }
  }

  // No Good Point Found
  if(bestNode == fatherNode){
    ROS_INFO_COND(verbose, "No New Node");
    return NO_NEW_NODE;
  }

  ROS_INFO_COND(verbose, "RRT Expansion Ended");
  return TRAJECTORY_FOUND;
}

double BezierExplorer::getGain(Eigen::Matrix<double, 3, 1> pose, double& yaw){
  geometry_msgs::Point query;
  query.x = pose(0);
  query.y = pose(1);
  query.z = pose(2);
  queryPublisher.publish(query);

  if(alphaMatrix_gp.size() == 0){
    return 0;
  }

  double actualGain = 0.0;
  std::vector<float> KS = kernel(pose);
  std::vector<float>::iterator alphaIter = alphaMatrix_gp.begin();
  for(std::vector<float>::iterator ksIter = KS.begin();
      ksIter != KS.end();
      ksIter++){
    
    actualGain += (*ksIter)*(*alphaIter);
    alphaIter++;
  }

  std::vector<float>::iterator xIter = xData_gp.begin();
  std::vector<float>::iterator yIter = yData_gp.begin();
  std::vector<float>::iterator zIter = zData_gp.begin();
  std::vector<float>::iterator yawIter = yawData_gp.begin();

  double distance_sq = INFINITY;
  double optimalYaw = 0.0;
  for( ; xIter != xData_gp.end(); ){
    double actualDistance_sq = pow((*xIter)-pose(0),2) + pow((*yIter)-pose(1),2) + pow((*zIter)-pose(2),2);
    if(actualDistance_sq < distance_sq){
      distance_sq = actualDistance_sq;
      optimalYaw = (*yawIter);
    }

    xIter++;
    yIter++;
    zIter++;
    yawIter++;
  }

  yaw = optimalYaw;
  return actualGain + 300;
}

std::vector<float> BezierExplorer::kernel(Eigen::Matrix<double, 3, 1> point){
  std::vector<float>::iterator xIter = xData_gp.begin();
  std::vector<float>::iterator yIter = yData_gp.begin();
  std::vector<float>::iterator zIter = zData_gp.begin();

  std::vector<float> KS;
  for( ; xIter != xData_gp.end(); ){
    float dx = pow((*xIter) - point(0), 2);
    float dy = pow((*yIter) - point(1), 2);
    float dz = pow((*zIter) - point(2), 2);

    float ks = exp(-(dx/(2*pow(hyperParams_gp[0], 2))))*exp(-(dy/(2*pow(hyperParams_gp[1], 2))))*exp(-(dz/(2*pow(hyperParams_gp[2], 2))));
    KS.push_back(ks);

    xIter++;
    yIter++;
    zIter++;
  }

  return KS;
}

Node* BezierExplorer::findFather(Eigen::Matrix<double, 3, 1> newPoint){
  double optimalDistance = INFINITY;
  Node* optimalNode;

  for(std::vector<Node*>::iterator treeIter = nodeTree.begin();
      treeIter != nodeTree.end();
      treeIter++){

    if(!(*treeIter)->isNodeActive()){
      continue;
    }

    double actualDistance = (newPoint - (*treeIter)->controlPoints_pose[(*treeIter)->controlPoints_pose.size()-1]).squaredNorm();
    if(actualDistance < optimalDistance){
      optimalDistance = actualDistance;
      optimalNode = *treeIter;
    }
  }

  return optimalNode;
}

void BezierExplorer::samplePoint(Eigen::Matrix<double, 3, 1>& newPoint, Eigen::Matrix<double, 3, 1> prevPoint, bool useMinRadius){
  // Sample Randomly Over a Sphere
  double sampleRadius_sq = pow(maxSamplingRad, 2);
  double minRadius_sq = pow(minSamplingRad, 2);
  
  do{
    if(useMinRadius){
      do{
        newPoint(0) = (((double)std::rand()/(double)RAND_MAX)*2*maxSamplingRad) - maxSamplingRad;
        newPoint(1) = (((double)std::rand()/(double)RAND_MAX)*2*maxSamplingRad) - maxSamplingRad;
        newPoint(2) = (((double)std::rand()/(double)RAND_MAX)*2*maxSamplingRad) - maxSamplingRad;
      } while(newPoint.squaredNorm() > sampleRadius_sq || newPoint.squaredNorm() < minRadius_sq);
    } else{
      do{
        newPoint(0) = (((double)std::rand()/(double)RAND_MAX)*2*maxSamplingRad) - maxSamplingRad;
        newPoint(1) = (((double)std::rand()/(double)RAND_MAX)*2*maxSamplingRad) - maxSamplingRad;
        newPoint(2) = (((double)std::rand()/(double)RAND_MAX)*2*maxSamplingRad) - maxSamplingRad;
      } while(newPoint.squaredNorm() > sampleRadius_sq);
    }

    newPoint(0) = newPoint(0) + prevPoint(0);
    newPoint(1) = newPoint(1) + prevPoint(1);
    newPoint(2) = newPoint(2) + prevPoint(2);
  } while(!insideBounds(newPoint));
}

void BezierExplorer::findBestDelta_rewire(Node* parentNode, Node* childNode,
                                          std::vector<Eigen::Matrix<double, 3, 1>>& controlPoints_pose,
                                          std::vector<double>& controlPoints_yaw,
                                          double& deltaTime, double& optimalCost){
  
  std::vector<Eigen::Matrix<double, 3, 1>> optimalControlPoints(6);
  std::vector<Eigen::Matrix<double, 3, 1>> actualControlPoints(6);
  std::vector<double> optimalControlPoints_yaw(4);
  std::vector<double> actualControlPoints_yaw(4);
  optimalCost = INFINITY;

  // Initialize New Points
  actualControlPoints[0] = parentNode->controlPoints_pose[parentNode->controlPoints_pose.size()-1];
  actualControlPoints[1] = parentNode->controlPoints_pose[parentNode->controlPoints_pose.size()-1];
  actualControlPoints[2] = parentNode->controlPoints_pose[parentNode->controlPoints_pose.size()-1];
  actualControlPoints[5] = childNode->controlPoints_pose[childNode->controlPoints_pose.size()-1];

  actualControlPoints_yaw[0] = parentNode->controlPoints_yaw[parentNode->controlPoints_yaw.size()-1];
  actualControlPoints_yaw[1] = parentNode->controlPoints_yaw[parentNode->controlPoints_yaw.size()-1];
  actualControlPoints_yaw[3] = childNode->controlPoints_yaw[childNode->controlPoints_yaw.size()-1];

  double postDelta = childNode->executionTime;

  Eigen::Matrix<double, 3, 1> ctrlPostP0 = childNode->controlPoints_pose[0];
  Eigen::Matrix<double, 3, 1> ctrlPostP1 = childNode->controlPoints_pose[1];
  Eigen::Matrix<double, 3, 1> ctrlPostP2 = childNode->controlPoints_pose[2];

  double ctrlPostP0_yaw = childNode->controlPoints_yaw[0];
  double ctrlPostP1_yaw = childNode->controlPoints_yaw[1];

  deltaTime = -1.0;
  for(std::vector<double>::iterator iter = times.begin(); iter != times.end(); iter++){
    // For All Times: Compute Continuity Conditions and Verify Feasibility
    double actualDelta = *iter;
    actualControlPoints[4] = actualControlPoints[5] - (actualDelta/postDelta)*(ctrlPostP1-ctrlPostP0);
    actualControlPoints[3] = (actualDelta/postDelta)*(actualDelta/postDelta)*(ctrlPostP2 - 2*ctrlPostP1 + ctrlPostP0) + 2*actualControlPoints[4] - actualControlPoints[5];

    actualControlPoints_yaw[2] = actualControlPoints_yaw[3] - (actualDelta/postDelta)*(ctrlPostP1_yaw-ctrlPostP0_yaw);

    // Check Collision
    Eigen::Matrix<double, 3, 1> convexHull_centre = Eigen::MatrixXd::Zero(3, 1);
    std::vector<Eigen::Matrix<double, 3, 1>> velocityControlPoints(5);
    std::vector<Eigen::Matrix<double, 3, 1>> accelerationControlPoints(4);
    bool velocityBoundViolated = false, accelerationBoundViolated = false;

    for(uint ii = 0; ii < actualControlPoints.size(); ii++){
      // Compute Convex Hull Centre
      convexHull_centre = convexHull_centre + actualControlPoints[ii];

      // Compute Velocities Control Points
      if(ii < actualControlPoints.size()-1){
        velocityControlPoints[ii] = (5/actualDelta)*(actualControlPoints[ii+1] - actualControlPoints[ii]);

        if(velocityControlPoints[ii].norm() > maxVelocity){
          ROS_INFO_COND(verbose_debug, "Velocity Bounds Violated With: %f", velocityControlPoints[ii].norm());
          velocityBoundViolated = true;
          break;
        }
      }

      // Compute Acceleration Control Points
      if(ii > 0 && ii < actualControlPoints.size()-1){
        accelerationControlPoints[ii-1] = (4/actualDelta)*(velocityControlPoints[ii] - velocityControlPoints[ii-1]);

        if(accelerationControlPoints[ii-1].norm() > maxAcceleration){
          ROS_INFO_COND(verbose_debug, "Acceleration Bounds Violated With: %f", accelerationControlPoints[ii-1].norm());
          accelerationBoundViolated = true;
          break;
        }
      }
    }

    if(velocityBoundViolated || accelerationBoundViolated){
      // Dynamic Bound Violated
      continue;
    }

    convexHull_centre = convexHull_centre/actualControlPoints.size();

    Eigen::Matrix<double, 3, 1> queryPoint;
    double comparingDistance = 0.0;
    bool collisionDetected = false;
    for(uint ii = 0; ii < actualControlPoints.size(); ii++){
      queryPoint = (convexHull_centre + actualControlPoints[ii])*0.5;
      comparingDistance = (queryPoint - actualControlPoints[ii]).norm() + safeDistance;

      //if((queryPoint-actualPosition).norm() > safeDistance){
        octomap::point3d query(queryPoint(0), queryPoint(1), queryPoint(2));
        if(collisionPoint(query, comparingDistance)){
          collisionDetected = true;
          break;
        }
      //}
    }

    if(collisionDetected){
      // Collision
      ROS_INFO_COND(verbose_debug, "Collission!");
      continue;
    }

    // Compute YAW Velocity CP
    std::vector<double> velocityControlPoints_yaw(3);
    for(uint ii = 0; ii < actualControlPoints_yaw.size(); ii++){
      if(ii < actualControlPoints_yaw.size()-1){
        velocityControlPoints_yaw[ii] = (3/actualDelta)*(actualControlPoints_yaw[ii+1] - actualControlPoints_yaw[ii]);
      }
    }

    // Everithing OK, Evaluate Cost
    Eigen::Matrix<double, 4, 1> A_x = Eigen::Matrix<double, 4, 1>(accelerationControlPoints[0](0), accelerationControlPoints[1](0), accelerationControlPoints[2](0), accelerationControlPoints[3](0));
    Eigen::Matrix<double, 4, 1> A_y = Eigen::Matrix<double, 4, 1>(accelerationControlPoints[0](1), accelerationControlPoints[1](1), accelerationControlPoints[2](1), accelerationControlPoints[3](1));
    Eigen::Matrix<double, 4, 1> A_z = Eigen::Matrix<double, 4, 1>(accelerationControlPoints[0](2), accelerationControlPoints[1](2), accelerationControlPoints[2](2), accelerationControlPoints[3](2));
    Eigen::Matrix<double, 4, 4> Ba = getBMatrix<Eigen::Matrix<double, 4, 4>>(actualDelta); 

    double cost_x = A_x.transpose()*Ba*A_x;
    double cost_y = A_y.transpose()*Ba*A_y;
    double cost_z = A_z.transpose()*Ba*A_z;

    Eigen::Matrix<double, 3, 1> V = Eigen::Matrix<double, 3, 1>(velocityControlPoints_yaw[0], velocityControlPoints_yaw[1], velocityControlPoints_yaw[2]);
    Eigen::Matrix<double, 3, 3> Bv = getBMatrix<Eigen::Matrix<double, 3, 3>>(actualDelta); 

    double cost_yaw = V.transpose()*Bv*V;

    double actualCost = actualDelta*rho_time + (cost_x + cost_y + cost_z)*rho_acc + cost_yaw*rho_wz;

    if(actualCost < optimalCost){
      optimalCost = actualCost;
      deltaTime = actualDelta;
      optimalControlPoints = actualControlPoints;
      optimalControlPoints_yaw = actualControlPoints_yaw;
    }
  }

  if(deltaTime > 0){
    controlPoints_pose = optimalControlPoints;
    controlPoints_yaw = optimalControlPoints_yaw;
  }
}

void BezierExplorer::findBestDelta(Node* parentNode, std::vector<Eigen::Matrix<double, 3, 1>>& newControlPoints_pose,
                                   std::vector<double>& newControlPoints_yaw, Eigen::Matrix<double, 3, 1> newPosition,
                                   double optimalYaw, double& deltaTime, double& optimalCost, bool finalPoint, bool trajReversed){

  std::vector<Eigen::Matrix<double, 3, 1>> optimalControlPoints(6);
  std::vector<Eigen::Matrix<double, 3, 1>> actualControlPoints(6);
  std::vector<double> optimalControlPoints_yaw(4);
  std::vector<double> actualControlPoints_yaw(4);
  optimalCost = INFINITY;
  
  // Previous Points
  Eigen::Matrix<double, 3, 1> ctrlPrevP3 = parentNode->controlPoints_pose[3];
  Eigen::Matrix<double, 3, 1> ctrlPrevP4 = parentNode->controlPoints_pose[4];
  Eigen::Matrix<double, 3, 1> ctrlPrevP5 = parentNode->controlPoints_pose[5];

  double yawPrevP2 = parentNode->controlPoints_yaw[2];
  double yawPrevP3 = parentNode->controlPoints_yaw[3];
  double previousDelta = parentNode->executionTime;

  if(trajReversed){
    ctrlPrevP3 = parentNode->controlPoints_pose[2];
    ctrlPrevP4 = parentNode->controlPoints_pose[1];
    ctrlPrevP5 = parentNode->controlPoints_pose[0];

    yawPrevP2 = parentNode->controlPoints_yaw[1];
    yawPrevP3 = parentNode->controlPoints_yaw[0];
  }

  // Initialize New Points
  actualControlPoints[0] = ctrlPrevP5;
  actualControlPoints[5] = newPosition;

  actualControlPoints_yaw[0] = yawPrevP3;
  actualControlPoints_yaw[3] = optimalYaw;

  deltaTime = -1.0;
  for(std::vector<double>::iterator iter = times.begin(); iter != times.end(); iter++){
    // For All Times: Compute Continuity Conditions and Verify Feasibility
    double actualDelta = *iter;

    // Velocity and Acceleration Continuity
    actualControlPoints[1] = (actualDelta/previousDelta)*(ctrlPrevP5-ctrlPrevP4) + actualControlPoints[0];
    actualControlPoints[2] = (actualDelta/previousDelta)*(actualDelta/previousDelta)*(ctrlPrevP5 - 2*ctrlPrevP4 + ctrlPrevP3) + 2*actualControlPoints[1] - actualControlPoints[0];

    // Only Velocity Continuity
    actualControlPoints_yaw[1] = (actualDelta/previousDelta)*(yawPrevP3-yawPrevP2) + actualControlPoints_yaw[0];

    if(!finalPoint){
      // Optimize Points
      Eigen::Matrix<double, 4, 1> PFixed_x = Eigen::Matrix<double, 4, 1>(actualControlPoints[0](0), actualControlPoints[1](0), actualControlPoints[2](0), actualControlPoints[5](0));
      Eigen::Matrix<double, 4, 1> PFixed_y = Eigen::Matrix<double, 4, 1>(actualControlPoints[0](1), actualControlPoints[1](1), actualControlPoints[2](1), actualControlPoints[5](1));
      Eigen::Matrix<double, 4, 1> PFixed_z = Eigen::Matrix<double, 4, 1>(actualControlPoints[0](2), actualControlPoints[1](2), actualControlPoints[2](2), actualControlPoints[5](2));
      Eigen::Matrix<double, 3, 1> PFixed_yaw = Eigen::Matrix<double, 3, 1>(actualControlPoints_yaw[0], actualControlPoints_yaw[1], actualControlPoints_yaw[3]);

      Eigen::Matrix<double, 4, 5> QA = getQAMatrix<Eigen::Matrix<double, 4, 5>>(actualDelta); 
      Eigen::Matrix<double, 5, 6> QV = getQVMatrix<Eigen::Matrix<double, 5, 6>>(actualDelta); 
      Eigen::Matrix<double, 4, 4> B = getBMatrix<Eigen::Matrix<double, 4, 4>>(actualDelta);
      Eigen::Matrix<double, 6, 6> C = getCMatrix<Eigen::Matrix<double, 6, 6>>();
      Eigen::Matrix<double, 6, 6> R = C.transpose()*QV.transpose()*QA.transpose()*B*QA*QV*C;

      Eigen::Matrix<double, 2, 2> R11_inv = R.block(0,0,2,2).fullPivHouseholderQr().inverse();
      Eigen::Matrix<double, 2, 4> R12 = R.block(0,2,2,4);
      Eigen::Matrix<double, 2, 1> PFree_x = -R11_inv*R12*PFixed_x;
      Eigen::Matrix<double, 2, 1> PFree_y = -R11_inv*R12*PFixed_y;
      Eigen::Matrix<double, 2, 1> PFree_z = -R11_inv*R12*PFixed_z;

      Eigen::Matrix<double, 3, 4> QV_yaw = getQVMatrix<Eigen::Matrix<double, 3, 4>>(actualDelta);
      Eigen::Matrix<double, 3, 3> B_yaw = getBMatrix<Eigen::Matrix<double, 3, 3>>(actualDelta);
      Eigen::Matrix<double, 4, 4> C_yaw = getCMatrix<Eigen::Matrix<double, 4, 4>>();

      Eigen::Matrix<double, 4, 4> R_yaw = C_yaw.transpose()*QV_yaw.transpose()*B_yaw*QV_yaw*C_yaw;
      Eigen::Matrix<double, 1, 3> R12_yaw = R_yaw.block(0,1,1,3);

      double R11_yaw_inv = 1/R_yaw(1,1);
      double PFree_yaw = -R11_yaw_inv*((R12_yaw*PFixed_yaw)(0));

      actualControlPoints[3] = Eigen::Matrix<double, 3, 1> (PFree_x(0), PFree_y(0), PFree_z(0));
      actualControlPoints[4] = Eigen::Matrix<double, 3, 1> (PFree_x(1), PFree_y(1), PFree_z(1));
      actualControlPoints_yaw[2] = PFree_yaw;

    } else{
      // To Have Zero Final Acceleration: We Require ctrlP5-ctrlP4 = ctrlP4-ctrlP3
      actualControlPoints[3] = actualControlPoints[5];
      actualControlPoints[4] = actualControlPoints[5];

      actualControlPoints_yaw[2] = actualControlPoints_yaw[3];
    }

    if(trajReversed){
      std::vector<Eigen::Matrix<double, 3, 1>> poseCP_reversed;
      std::vector<double> yawCP_reversed;

      for(uint ii = 0; ii < actualControlPoints.size(); ii++){
        poseCP_reversed.push_back(actualControlPoints[actualControlPoints.size()-1-ii]);
      }

      for(uint ii = 0; ii < actualControlPoints_yaw.size(); ii++){
        yawCP_reversed.push_back(actualControlPoints_yaw[actualControlPoints_yaw.size()-1-ii]);
      }
    }

    // Check Collision
    Eigen::Matrix<double, 3, 1> convexHull_centre = Eigen::MatrixXd::Zero(3, 1);
    std::vector<Eigen::Matrix<double, 3, 1>> velocityControlPoints(5);
    std::vector<Eigen::Matrix<double, 3, 1>> accelerationControlPoints(4);
    bool velocityBoundViolated = false, accelerationBoundViolated = false;

    for(uint ii = 0; ii < actualControlPoints.size(); ii++){
      // Compute Convex Hull Centre
      convexHull_centre = convexHull_centre + actualControlPoints[ii];

      // Compute Velocities Control Points
      if(ii < actualControlPoints.size()-1){
        velocityControlPoints[ii] = (5/actualDelta)*(actualControlPoints[ii+1] - actualControlPoints[ii]);

        if(velocityControlPoints[ii].norm() > maxVelocity){
          ROS_INFO_COND(verbose_debug, "Velocity Bounds Violated With: %f", velocityControlPoints[ii].norm());
          velocityBoundViolated = true;
          break;
        }
      }

      // Compute Acceleration Control Points
      if(ii > 0 && ii < actualControlPoints.size()-1){
        accelerationControlPoints[ii-1] = (4/actualDelta)*(velocityControlPoints[ii] - velocityControlPoints[ii-1]);

        if(accelerationControlPoints[ii-1].norm() > maxAcceleration){
          ROS_INFO_COND(verbose_debug, "Acceleration Bounds Violated With: %f", accelerationControlPoints[ii-1].norm());
          accelerationBoundViolated = true;
          break;
        }
      }
    }

    if(velocityBoundViolated || accelerationBoundViolated){
      // Dynamic Bound Violated
      continue;
    }

    convexHull_centre = convexHull_centre/actualControlPoints.size();

    Eigen::Matrix<double, 3, 1> queryPoint;
    double comparingDistance = 0.0;
    bool collisionDetected = false;
    for(uint ii = 1; ii < actualControlPoints.size(); ii++){
      queryPoint = (convexHull_centre + actualControlPoints[ii])*0.5;
      comparingDistance = (queryPoint - actualControlPoints[ii]).norm() + safeDistance;

      //if((queryPoint-actualPosition).norm() > safeDistance){
        octomap::point3d query(queryPoint(0), queryPoint(1), queryPoint(2));
        if(collisionPoint(query, comparingDistance)){
          collisionDetected = true;
          break;
        }
      //}
    }

    if(collisionDetected){
      // Collision
      ROS_INFO_COND(verbose_debug, "Collision");
      continue;
    }

    // Everithing OK, Evaluate Cost
    std::vector<double> velocityControlPoints_yaw(3);
    for(uint ii = 0; ii < actualControlPoints_yaw.size()-1; ii++){
      velocityControlPoints_yaw[ii] = (5/actualDelta)*(actualControlPoints_yaw[ii+1] - actualControlPoints_yaw[ii]);
    }

    Eigen::Matrix<double, 4, 1> A_x = Eigen::Matrix<double, 4, 1>(accelerationControlPoints[0](0), accelerationControlPoints[1](0), accelerationControlPoints[2](0), accelerationControlPoints[3](0));
    Eigen::Matrix<double, 4, 1> A_y = Eigen::Matrix<double, 4, 1>(accelerationControlPoints[0](1), accelerationControlPoints[1](1), accelerationControlPoints[2](1), accelerationControlPoints[3](1));
    Eigen::Matrix<double, 4, 1> A_z = Eigen::Matrix<double, 4, 1>(accelerationControlPoints[0](2), accelerationControlPoints[1](2), accelerationControlPoints[2](2), accelerationControlPoints[3](2));
    Eigen::Matrix<double, 4, 4> B = getBMatrix<Eigen::Matrix<double, 4, 4>>(actualDelta); 

    Eigen::Matrix<double, 3, 1> A_w = Eigen::Matrix<double, 3, 1>(velocityControlPoints_yaw[0], velocityControlPoints_yaw[1], velocityControlPoints_yaw[3]);
    Eigen::Matrix<double, 3, 3> B_w = getBMatrix<Eigen::Matrix<double, 3, 3>>(actualDelta); 

    double cost_x = A_x.transpose()*B*A_x;
    double cost_y = A_y.transpose()*B*A_y;
    double cost_z = A_z.transpose()*B*A_z;
    double cost_w = A_w.transpose()*B_w*A_w;
    double actualCost = actualDelta*rho_time + (cost_x + cost_y + cost_z)*rho_acc + cost_w*rho_wz;

    if(actualCost < optimalCost){
      optimalCost = actualCost;
      deltaTime = actualDelta;
      optimalControlPoints = actualControlPoints;
      optimalControlPoints_yaw = actualControlPoints_yaw;
    }
  }

  if(deltaTime > 0){
    newControlPoints_pose = optimalControlPoints;
    newControlPoints_yaw = optimalControlPoints_yaw;
  }
}

bool BezierExplorer::insideBounds(Eigen::Matrix<double, 3, 1> position){
  if(position(0) > mapBoundMaxX || position(0) < mapBoundMinX)
    return false;

  if(position(1) > mapBoundMaxY || position(1) < mapBoundMinY)
    return false;

  if(position(2) > mapBoundMaxZ || position(2) < mapBoundMinZ)
    return false;

  return true;
}

void BezierExplorer::planSafeTrajectory(Node* node, std::vector<Eigen::Matrix<double, 3, 1>>& controlPoints_pose,
                                        std::vector<double>& controlPoints_yaw, double& deltaTime, double& optimalCost, bool trajReversed){
  std::shared_ptr<octomap::OcTree> ot = ot_;
  Eigen::Matrix<double, 3, 1> newPosition;
  deltaTime = -1.0;
  int iterNum = 0;

  do{
    octomap::point3d query;
    octomap::OcTreeNode* result;

    do{
      if(trajReversed){
        samplePoint(newPosition, node->controlPoints_pose[0], false);
      } else{
        samplePoint(newPosition, node->controlPoints_pose[node->controlPoints_pose.size()-1], false);
      }

      query = octomap::point3d(newPosition(0), newPosition(1), newPosition(2));
      result = ot_->search(query);

      if(!result){
        ROS_INFO_COND(verbose_debug, "Not Observed");
      }
    } while(!result || collisionPoint(query, safeDistance*1.5));

    if(trajReversed){
      findBestDelta(node, controlPoints_pose, controlPoints_yaw, newPosition, node->controlPoints_yaw[0], deltaTime, optimalCost, true, trajReversed);
    } else{
      findBestDelta(node, controlPoints_pose, controlPoints_yaw, newPosition, node->controlPoints_yaw[node->controlPoints_yaw.size()-1], deltaTime, optimalCost, true, false);
    }

    iterNum ++;
  } while(deltaTime < 0 && iterNum < maxIterNum);
}

void BezierExplorer::publishTraj(bezier_exploration::BezierTraj msg){
  Eigen::Matrix<double, 3, 6> actualTraj;
  Eigen::Matrix<double, 3, 6> safeTraj;

  for(uint ii = 0; ii < msg.x.size(); ii++){
    actualTraj(0, ii) = msg.x[ii];
    actualTraj(1, ii) = msg.y[ii];
    actualTraj(2, ii) = msg.z[ii];
  }

  for(uint ii = 0; ii < msg.x_safe.size(); ii++){
    safeTraj(0, ii) = msg.x_safe[ii];
    safeTraj(1, ii) = msg.y_safe[ii];
    safeTraj(2, ii) = msg.z_safe[ii];
  }

  markerTraj.header.frame_id = "map";
  markerTraj.header.stamp = ros::Time::now();
  markerTraj.ns = "RRT-Traj";
  markerTraj.action = visualization_msgs::Marker::ADD;
  markerTraj.pose.orientation.w = 1.0;
  markerTraj.id += 1;
  markerTraj.type = visualization_msgs::Marker::SPHERE_LIST;
  markerTraj.scale.x = 0.03;
  markerTraj.scale.y = 0.03;
  markerTraj.scale.z = 0.03;
  markerTraj.color.b = 1.0;
  markerTraj.color.r = 0.0;
  markerTraj.color.a = 1.0;

  geometry_msgs::Point p;
  Eigen::Matrix<double, 6, 1> B;
  for(double tt = 0; tt <= 1; tt += 0.01){
    B << pow(1-tt, 5), 5*tt*pow(1-tt,4), 10*pow(tt,2)*pow(1-tt,3), 10*pow(tt,3)*pow(1-tt,2), 5*pow(tt,4)*(1-tt), pow(tt,5);
    Eigen::Matrix<double, 3, 1> actualPoint = actualTraj*B;

    p.x = actualPoint(0);
    p.y = actualPoint(1);
    p.z = actualPoint(2);

    markerTraj.points.push_back(p);
  }

  trajVisPublisher.publish(markerTraj);
  markerTraj.points.clear();
  markerTraj.points.shrink_to_fit();

  markerTraj.header.frame_id = "map";
  markerTraj.header.stamp = ros::Time::now();
  markerTraj.ns = "RRT-Traj-Safe";
  markerTraj.action = visualization_msgs::Marker::ADD;
  markerTraj.pose.orientation.w = 1.0;
  markerTraj.id += 1;
  markerTraj.type = visualization_msgs::Marker::SPHERE_LIST;
  markerTraj.scale.x = 0.03;
  markerTraj.scale.y = 0.03;
  markerTraj.scale.z = 0.03;
  markerTraj.color.r = 1.0;
  markerTraj.color.b = 0.0;
  markerTraj.color.a = 1.0;
  
  for(double tt = 0; tt <= 1; tt += 0.01){
    B << pow(1-tt, 5), 5*tt*pow(1-tt,4), 10*pow(tt,2)*pow(1-tt,3), 10*pow(tt,3)*pow(1-tt,2), 5*pow(tt,4)*(1-tt), pow(tt,5);
    Eigen::Matrix<double, 3, 1> actualPoint = safeTraj*B;

    p.x = actualPoint(0);
    p.y = actualPoint(1);
    p.z = actualPoint(2);

    markerTraj.points.push_back(p);
  }

  trajVisPublisher.publish(markerTraj);
  markerTraj.points.clear();
  markerTraj.points.shrink_to_fit();
}

void BezierExplorer::publishTree(){
  visualization_msgs::Marker markerTree, branchTree;  

  markerTree.header.frame_id = branchTree.header.frame_id = "map";
  markerTree.header.stamp = branchTree.header.stamp = ros::Time::now();
  markerTree.ns = branchTree.ns = "RRT";
  markerTree.action = branchTree.action = visualization_msgs::Marker::ADD;
  markerTree.pose.orientation.w = branchTree.pose.orientation.w = 1.0;

  markerTree.id = 0;
  branchTree.id = 1;

  markerTree.type = visualization_msgs::Marker::SPHERE_LIST;
  branchTree.type = visualization_msgs::Marker::LINE_LIST;

  markerTree.scale.x = 0.03;
  markerTree.scale.y = 0.03;
  markerTree.scale.z = 0.03;
  branchTree.scale.x = 0.01;
  branchTree.scale.y = 0.01;
  branchTree.scale.z = 0.01;

  markerTree.color.g = 1.0;
  markerTree.color.a = 1.0;
  branchTree.color.r = 1.0;
  branchTree.color.a = 1.0;

  geometry_msgs::Point p, pp;
  for(std::vector<Node*>::iterator iter = nodeTree.begin();
      iter != nodeTree.end(); iter++){
    
    Node* actualNode = *iter;

    p.x = actualNode->controlPoints_pose[actualNode->controlPoints_pose.size()-1](0);
    p.y = actualNode->controlPoints_pose[actualNode->controlPoints_pose.size()-1](1);
    p.z = actualNode->controlPoints_pose[actualNode->controlPoints_pose.size()-1](2);

    if(actualNode->nodeParent != NULL){
      pp.x = actualNode->nodeParent->controlPoints_pose[actualNode->controlPoints_pose.size()-1](0);
      pp.y = actualNode->nodeParent->controlPoints_pose[actualNode->controlPoints_pose.size()-1](1);
      pp.z = actualNode->nodeParent->controlPoints_pose[actualNode->controlPoints_pose.size()-1](2);

      branchTree.points.push_back(p);
      branchTree.points.push_back(pp);
    }

    markerTree.points.push_back(p);
  }

  treePublisher.publish(markerTree);
  branchPublisher.publish(branchTree);
}

bool BezierExplorer::collisionPoint(octomap::point3d query, double distance){
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double dd = distance/2;
  octomap::point3d start(query.x() - dd, query.y() - dd, query.z() - dd);
  octomap::point3d end(query.x() + dd, query.y() + dd, query.z() + dd);

  for(octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(start, end);
      it != ot->end_leafs_bbx(); it++){
      
    if(it->getOccupancy() > 0.6){
      // Occupied!
      return true;
    }
  }

  return false;
}