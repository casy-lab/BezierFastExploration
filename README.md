# Bezier Fast Exploration
## Algorithm Features Overview
<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration100s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration200s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration300s.png" width = 30% float="left" />
</p>

BezierFastExploration is a flexible framework for three-dimensional next-best-view Gaussian process aided exploration design.
The repository contains all the necessary code to build your own exploration algorithm with your own custom information gain function.
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
* 

## Authors
  * Lorenzo Gentilini - PhD Student
    * Email: lorenzo.gentilini6@unibo.it
  * Dario Mengoli - PhD Student
    * Email: dario.mengoli2@unibo.it
  * Lorenzo Marconi - Full Professor
    * Email: lorenzo.marconi@unibo.it
    
## References
   For the details of the work, please refer to the papers:
   * Permament Link form arXiv

# Setup
## Prerequisites and Dependencies
This code has been tested on ROS Melodic and ROS Noetic.

Install ROS following the online tutorials.

**Dependencies**

Octomap, Eigen

Python numpy, scipy, rtree

A depth camera integrated with ros, that publish a pointcloud2 topic.

## Building the Code
Clone the repository in your workspace
```
git clone https://github.com/casy-lab/BezierFastExploration.git bezier_exploration
```
then run
```
catkin_make
```

## Run the Exploration
- Start the px4 subsystem (either real or using software in the loop simulation)
- Start the RGB-D camera integration (to obtain the pointcloud)
- Start the exploration node
```
roslaunch bezier_exploration exploration.launch
```
- Start the gain regressor node
```
rosrun bezier_exploration gain_regressor.py
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