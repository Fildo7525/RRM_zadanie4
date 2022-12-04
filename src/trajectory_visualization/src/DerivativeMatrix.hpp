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

extern std::fstream klby;
extern std::vector<std::vector<double>> data;


Eigen::MatrixXd sixBySixDerivative(int t0, int t2);
Eigen::MatrixXd nineByNineDerivative(int t0, int t1, int t2);

Eigen::VectorXd calculateAParams(double startTime, double endTime, Eigen::MatrixXd &&pointFinal);
Eigen::MatrixXd calcultateData(Eigen::MatrixXd a, const double time);
void wiriteTrajectory(moveit_msgs::RobotTrajectory &trajectory,
					  double startTime,
					  double endTime,
					  const Eigen::MatrixXd &jointOne,
					  const Eigen::MatrixXd &jointThree);

