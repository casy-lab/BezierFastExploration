# Bezier Fast Exploration
<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration100s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration200s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Exploration300s.png" width = 30% float="left" />
</p>

BezierFastExploration is a flexible framework for next-best-view exploration design.
In particular, this repository contains all procedures required by an

Autonomous exploration of large unknown areas using Bézier curves trajectories.

<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/mappedVolume.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/traveledDistance.png" width = 30% float="left" />
</p>

## Prerequisites
This code has been tested on ROS Melodic and ROS Noetic.

Install ROS following the online tutorials.

**Dependencies**

Octomap, Eigen

Python numpy, scipy, rtree

A depth camera integrated with ros, that publish a pointcloud2 topic.

## Building node
Clone the repository in your workspace
```
git clone https://github.com/casy-lab/BezierFastExploration.git bezier_exploration
```
then run
```
catkin_make
```

## Launching the exploration
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
   
   
## Images
<p float="left" align="center">
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/10s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/57s.png" width = 30% float="left" />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/167s.png" width = 30% float="left" />
</p>
