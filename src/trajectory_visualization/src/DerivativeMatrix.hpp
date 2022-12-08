#pragma once

#include <QSplineSeries>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <cmath>
#include <vector>

#include "Eigen/src/Core/Matrix.h"


#define T0 0
#define T1 1
#define T2 2
#define T3 3
#define T4 4
#define T5 5
#define T6 9

extern QtCharts::QSplineSeries *zPose;
extern QtCharts::QSplineSeries *zSpeed;
extern QtCharts::QSplineSeries *zAcc;
extern QtCharts::QSplineSeries *yPose;
extern QtCharts::QSplineSeries *ySpeed;
extern QtCharts::QSplineSeries *yAcc;
extern QtCharts::QSplineSeries *zRotation;

Eigen::MatrixXd fourByFourDerivative(int t0, int t2);
Eigen::MatrixXd fiveByFiveDerivative(int t0, int t1, int t2);
Eigen::MatrixXd sixBySixDerivative(int t0, int t2);

Eigen::VectorXd calculateAParams(double startTime, double endTime, Eigen::MatrixXd &&pointFinal, int matrixSize, double midTime = 0);
Eigen::MatrixXd calcultateData(Eigen::MatrixXd a, const double time);
void writeTrajectory(moveit_msgs::RobotTrajectory &trajectory,
					  double startTime,
					  double endTime,
					  std::function<std::vector<double>(double)> motion,
					  const Eigen::MatrixXd &stop);
std::vector<double> solutionFromIkconst(const Eigen::Vector3d &position, double rx, double ry, double rz);
void writeToChart(int zyr, Eigen::MatrixXd data, double t);

