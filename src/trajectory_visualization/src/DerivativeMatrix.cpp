#include "DerivativeMatrix.hpp"

std::fstream klby("/home/fildo/Zad4/src/trajectory_visualization/hodnoty.txt", std::ios::out);
std::vector<std::vector<double>> data(8);

Eigen::MatrixXd sixBySixDerivative(int t0, int t2)
{
	Eigen::MatrixXd point(6, 6);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),	  std::pow(t0, 4),	  std::pow(t0, 5),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),  4*std::pow(t0, 3),  5*std::pow(t0, 4),
			 0,	   0,				2,				 6*t0, 12*std::pow(t0, 2), 20*std::pow(t0, 3),
			 1,   t2, std::pow(t2, 2),	  std::pow(t2, 3),	  std::pow(t2, 4),	  std::pow(t2, 5),
			 0,    1,			 2*t2,  3*std::pow(t2, 2),  4*std::pow(t2, 3),  5*std::pow(t2, 4),
			 0,    0,				2,				 6*t2, 12*std::pow(t2, 2), 20*std::pow(t2, 3);
	return point;
}

Eigen::MatrixXd nineByNineDerivative(int t0, int t1, int t2)
{
	Eigen::MatrixXd point(9, 9);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),	  std::pow(t0, 4),	  std::pow(t0, 5),	  std::pow(t0, 6),	  std::pow(t0, 7),	  std::pow(t0, 8),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),  4*std::pow(t0, 3),  5*std::pow(t0, 4),	6*std::pow(t0, 5),	7*std::pow(t0, 6),	8*std::pow(t0, 7),
			 0,    0,				1,				 6*t0, 12*std::pow(t0, 2), 20*std::pow(t0, 3), 30*std::pow(t0, 4), 42*std::pow(t0, 5), 56*std::pow(t0, 6),
			 1,   t1, std::pow(t1, 2),	  std::pow(t1, 3),	  std::pow(t1, 4),	  std::pow(t1, 5),	  std::pow(t1, 6),	  std::pow(t1, 7),	  std::pow(t1, 8),
			 0,    1,			 2*t1,  3*std::pow(t1, 2),  4*std::pow(t1, 3),  5*std::pow(t1, 4),	6*std::pow(t1, 5),	7*std::pow(t1, 6),	8*std::pow(t1, 7),
			 0,    0,				1,				 6*t1, 12*std::pow(t1, 2), 20*std::pow(t1, 3), 30*std::pow(t1, 4), 42*std::pow(t1, 5), 56*std::pow(t1, 6),
			 1,   t2, std::pow(t2, 2),	  std::pow(t2, 3),	  std::pow(t2, 4),	  std::pow(t2, 5),	  std::pow(t2, 6),	  std::pow(t2, 7),	  std::pow(t2, 8),
			 0,    1,			 2*t2,  3*std::pow(t2, 2),  4*std::pow(t2, 3),  5*std::pow(t2, 4),	6*std::pow(t2, 5),	7*std::pow(t2, 6),	8*std::pow(t2, 7),
			 0,    0,				1,				 6*t2, 12*std::pow(t2, 2), 20*std::pow(t2, 3), 30*std::pow(t2, 4), 42*std::pow(t2, 5), 56*std::pow(t2, 6);
	return point;
}

Eigen::VectorXd calculateAParams(double startTime, double endTime, Eigen::MatrixXd &&pointFinal)
{
	Eigen::MatrixXd pointOne = sixBySixDerivative(startTime, endTime);
	Eigen::VectorXd pointOneAParams;

	pointOneAParams = pointOne.inverse() * pointFinal;
    ROS_INFO_STREAM("pointAParams = \n" << pointOneAParams);
	return pointOneAParams;
}

std::vector<double> calculateRobotJoint3Variables(double t, Eigen::VectorXd a)
{
	std::vector<double> q;
	// Position
	q.push_back(a[0] + a[1]*std::pow(t,1) + a[2]*std::pow(t,2) + a[3]*std::pow(t,3) + a[4]*std::pow(t,4) + a[5]*std::pow(t,5));
	// Velocity
	q.push_back(a[1] + 2*a[2]*std::pow(t,1) + 3*a[3]*std::pow(t,2) + 4*a[4]*std::pow(t,3) + 5*a[5]*std::pow(t,4));
	// Acceleration
	q.push_back(2*a[2] + 6*a[3]*std::pow(t,1) + 12*a[4]*std::pow(t,2) + 20*a[5]*std::pow(t,3));
	// Yank
	q.push_back(6*a[3] + 24*a[4]*std::pow(t,1) + 60*a[5]*std::pow(t,2));
	return q;
}

Eigen::MatrixXd calcultateData(Eigen::MatrixXd a, const double time)
{
	Eigen::MatrixXd dataPoint(4,1);
	double entry = 0;

	// Position
	for (size_t i = 0; i < a.size(); i++) {
		entry += a(i) * std::pow(time, i);
	}
	dataPoint(0) = entry;

	// Velocity
	entry = 0;
	for (size_t i = 1; i < a.size(); i++) {
		entry += i*a(i) * std::pow(time, i-1);
	}
	dataPoint(1) = entry;

	// Acceleration
	entry = 0;
	for (size_t i = 2; i < a.size(); i++) {
		entry += i*(i-1)*a(i) * std::pow(time, i-2);
	}
	dataPoint(2) = entry;

	// Yank
	entry = 0;
	for (size_t i = 3; i < a.size(); i++) {
		entry += i*(i-1)*(i-2)*a(i) * std::pow(time, i-3);
	}
	dataPoint(3) = entry;

	return dataPoint;
}

void wiriteTrajectory(moveit_msgs::RobotTrajectory &trajectory,
					  double startTime,
					  double endTime,
					  const Eigen::MatrixXd &jointOne,
					  const Eigen::MatrixXd &jointThree)
{
	ROS_INFO_STREAM("Calculation " << startTime << " - " << endTime << " second");
	for (double t = startTime; t <= endTime; t += 0.1) {

		// Vytvorenie prejazdoveho bodu
		trajectory_msgs::JointTrajectoryPoint point;

		// Robot ma 6 klbov
		point.positions.resize(6);
		point.velocities.resize(6);
		point.accelerations.resize(6);


		// Joint 1
		auto jointOneData = calcultateData(jointOne, t);
		point.positions[0] = jointOneData(0);
		point.velocities[0] = jointOneData(1);
		point.accelerations[0] = jointOneData(2);
		// ROS_INFO_STREAM("J1 at " << t << "\n" << jointOneData);
		ROS_INFO_STREAM("J3 at " << t << "\n" << jointOneData);
		data[0].push_back(jointOneData(0));
		data[1].push_back(jointOneData(1));
		data[2].push_back(jointOneData(2));
		data[3].push_back(jointOneData(3));


		// Klb 2
		point.positions[1] = 0;
		point.velocities[1] = 0;
		point.accelerations[1] = 0;

		// Joint 3
		auto jointThreeData = calcultateData(jointThree, t);
		point.positions[2] = jointThreeData(0);
		point.velocities[2] = jointThreeData(1);
		point.accelerations[2] = jointThreeData(2);
		ROS_INFO_STREAM("J3 at " << t << "\n" << jointThreeData);
		data[4].push_back(jointThreeData(0));
		data[5].push_back(jointThreeData(1));
		data[6].push_back(jointThreeData(2));
		data[7].push_back(jointThreeData(3));

		// Klb 4
		point.positions[3] = 0;
		point.velocities[3] = 0;
		point.accelerations[3] = 0;

		// Klb 5
		point.positions[4] = 0;
		point.velocities[4] = 0;
		point.accelerations[4] = 0;

		// Klb 6
		point.positions[5] = 0;
		point.velocities[5] = 0;
		point.accelerations[5] =0;

		ROS_INFO_STREAM('\n');


		// Vlozenie casu prejazdu
		point.time_from_start = ros::Duration(t);

		// VLozenie bodu do trajektorie
		trajectory.joint_trajectory.points.push_back(point);
	}
}

