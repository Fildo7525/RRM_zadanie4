#pragma once

#include <qsplineseries.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>

#include <QSplineSeries>

#define T0 0
#define T1 1
#define T2 4

extern QtCharts::QSplineSeries *position1;
extern QtCharts::QSplineSeries *speed1;
extern QtCharts::QSplineSeries *acceleration1;
extern QtCharts::QSplineSeries *yank1;
extern QtCharts::QSplineSeries *position2;
extern QtCharts::QSplineSeries *speed2;
extern QtCharts::QSplineSeries *acceleration2;
extern QtCharts::QSplineSeries *yank2;

Eigen::MatrixXd fourByFourDerivative(int t0, int t2);
Eigen::MatrixXd fiveByFiveDerivative(int t0, int t1, int t2);
Eigen::MatrixXd sixBySixDerivative(int t0, int t2);

Eigen::VectorXd calculateAParams(double startTime, double endTime, Eigen::MatrixXd &&pointFinal, int matrixSize, double midTime = 0);
Eigen::MatrixXd calcultateData(Eigen::MatrixXd a, const double time);
void wiriteTrajectory(moveit_msgs::RobotTrajectory &trajectory,
					  double startTime,
					  double endTime,
					  const Eigen::MatrixXd &jointOne,
					  const Eigen::MatrixXd &jointThree);

