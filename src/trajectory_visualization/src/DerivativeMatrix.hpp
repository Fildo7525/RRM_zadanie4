#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>
#include <fstream>

#define T0 0
#define T1 2
#define T2 4

Eigen::MatrixXd sixBySixDerivative(int t0, int t2);
Eigen::MatrixXd nineByNineDerivative(int t0, int t1, int t2);

