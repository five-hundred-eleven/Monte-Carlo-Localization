# Monte Carlo Localization

A simple implementation of the MCL particle filter algorithm, as outlined by Thrun 
and Norvig in their online [Intro to AI](www.ai-class.com) course.

This program is designed to be used under [ROS](www.ros.org) (Robot Operating
System). Specifically, use the 
[UML_HMM package](http://www.cs.uml.edu/ecg/pub/uploads/MRspr12/uml_hmm.tar.gz)
. Its is a simulation of a 1 dimensional world, where the robot can move left 
or right down a hall and detect if it is moving past a door or a wall. The 
robot's motor and sensors are imperfect, so the robot moves variable distances
per time stamp and sometimes gives bad sensor readings. 

