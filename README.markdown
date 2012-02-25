# Monte Carlo Localization
## Background

A simple implementation of the MCL particle filter algorithm, as outlined by Thrun 
and Norvig in their online [Intro to AI](http://www.ai-class.com) course.

This program is designed to be used under [ROS](http://www.ros.org) (Robot Operating
System). Specifically, use the 
[UML_HMM](http://www.cs.uml.edu/ecg/pub/uploads/MRspr12/uml_hmm.tar.gz)
package. Its is a simulation of a 1 dimensional world, where the robot can move left 
or right down a hall and detect if it is moving past a door or a wall. The 
robot's motor and sensors are imperfect, so the robot moves variable distances
per time stamp and sometimes gives bad sensor readings. 

# Particle.py

This is the MCL implementation. If you have ROS setup and the UML_HMM package 
installed, move particle.py into the nodes/ directory and run.

# Driver-Right.py and Driver-Left.py

An alternative algorithm for the same scenario. This implements the bin 
localization algorithm, which is simpler for this purpose, but very 
computationally expensive in more complex scenarios. 
