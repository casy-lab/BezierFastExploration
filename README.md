# Bezier Fast Exploration
Autonomous exploration of large unknown areas using Bézier curves trajectories.

## Prerequisites
This code has been tested on ROS Melodic and ROS Noetic.

Install ROS following the online tutorials.

*Dependencies*



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
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/167s.png" width = 100% height = 50% />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/Full.png" width = 100% height = 50% />
   <img src="https://github.com/casy-lab/BezierFastExploration/blob/master/support_files/simMap_2.jpg" width = 100% height = 50% />
   
### 1.2 MAVROS and GeographicLib
Install ROS repositories for mavros:
```
sudo apt-get install ros-*ros_distro*-mavros ros-*ros_distro*-mavros-extras
```
Install [GeographicLib](https://geographiclib.sourceforge.io/) dataset by running:
```
cd mavros/mavros/scripts
sudo ./install_geographiclib_datasets.sh
```
