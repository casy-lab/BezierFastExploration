# Bezier Fast Exploration
## Algorithm Features Overview
<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration100s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration200s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration300s.png" width = 30% float="left" />
</p>

BezierFastExploration is a flexible framework for three-dimensional next-best-view Gaussian process aided exploration design.
This repository contains all the necessary code to build your own exploration algorithm with your own custom information gain function.
The implemented algorithm iteratively builds and maintains a Rapidly-Exploring Random Tree (RRT) by randomly sampling feasible trajectories.
The information gain of each trajectory is evaluated via sparse ray-casting and Gaussian process regression.
The algorithm employs a Bézier curve parameterisation to plan feasible collision-free high informative trajectories through the exploring unknown environment.
For more information please see our paper.

<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/mappedVolume.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/traveledDistance.png" width = 30% float="left" />
</p>

# Credits
## Paper and Video
If you find this package useful for your research, please consider citing our paper:
* L. Gentilini, D. Mengoli, and L. Marconi. **Direct Bézier-Based Trajectory Planner for Improved Local Exploration of Unknown Environments**. 
  arXiv:2203.00968, 2022. ([Paper](https://arxiv.org/pdf/2203.00968.pdf))

## Authors
  * Lorenzo Gentilini - PhD Student
    * Email: lorenzo.gentilini6@unibo.it
  * Dario Mengoli - PhD Student
    * Email: dario.mengoli2@unibo.it
  * Lorenzo Marconi - Full Professor
    * Email: lorenzo.marconi@unibo.it

# Setup the Package
## Prerequisites and Dependencies
This code has been tested on ROS Melodic and ROS Noetic.

In order to build and run this package, please follow the ROS install online tutorial ([ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu))

Install Octomap and Eigen:
```
sudo apt-get install ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-octomap-msgs libeigen3-dev
```
Install numpy, scipy, and rtree:
```
pip3 install numpy scipy rtree
```

## Building the Code
Clone the repository in your workspace:
```
git clone https://github.com/casy-lab/BezierFastExploration.git bezier_exploration
```
Then run the following commands:
```
cd bezier_exploration
catkin_make
```

## Run the Exploration
- Start the PX4 subsystem (either real or using software in the loop simulation)
- Start the RGB-D camera integration (to obtain the pointcloud)
- Start the exploration node:
```
roslaunch bezier_exploration exploration.launch
```
   
## Real-World Tests
<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/20s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/57s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/167s.png" width = 30% float="left" />
</p>

<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/testScenario_h.jpg" width = 91% float="left" />
</p>