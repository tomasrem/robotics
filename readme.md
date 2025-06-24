# Robotics 
This repository contains all the code used for robotics and python assignment . 
All the changes of code are tracked and each commit has short message describing them. 

Provided code is for simulation of robotic car that follows black line and is using dijkstra shortest path algorithm to get the to deistantion point.
In esp32 is neccesary to provide the robot with starting and goal position Point label and starting direction eg U (up) L (left) R (right) D (down) and in webots its neccesarry to edit the starting position x,y,pos coordinates for the odomettry based localisation. 

ESP 32 code is located at main.py 
Webots controller is located at webots_controller.py 
Visualisation of the planned path is at visualisation.py , please run this in VS code or from command line 
Code was run with python 3.11.5 and micropython 1.25 , and webots version  R2025a