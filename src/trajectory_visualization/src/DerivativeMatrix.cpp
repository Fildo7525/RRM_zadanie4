#include "DerivativeMatrix.hpp"
#include <limits>
#include <vector>

QtCharts::QSplineSeries *zPose = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *zSpeed = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *zAcc = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *yPose = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *ySpeed = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *yAcc = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *zRotation = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *zRotationS = new QtCharts::QSplineSeries();
QtCharts::QSplineSeries *zRotationA = new QtCharts::QSplineSeries();
double n_zPose = 1.6;
double n_zSpeed = 0;
double n_zAcc = 0;
double n_yPose = 0;
double n_ySpeed = 0;
double n_yAcc = 0;
double n_zRotation = 0;
double n_zRotationS = 0;
double n_zRotationA = 0;
double xLast;
double yLast;
double zLast;
double rxLast;
double ryLast;
double rzLast;
std::vector<double> lastSolution;
std::vector<double> SolutionCalculationHelper(6, 0);

Eigen::MatrixXd fourByFourDerivative(int t0, int t2)
{
	Eigen::MatrixXd point(4, 4);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),
			 1,   t2, std::pow(t2, 2),	  std::pow(t2, 3),
			 0,    1,			 2*t2,  3*std::pow(t2, 2);
	return point;
}

Eigen::MatrixXd fiveByFiveDerivative(int t0, int t1, int t2)
{
	Eigen::MatrixXd point(5, 5);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),	  std::pow(t0, 4),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),  4*std::pow(t0, 3),
			 1,   t1, std::pow(t1, 2),	  std::pow(t1, 3),	  std::pow(t1, 4),
			 1,   t2, std::pow(t2, 2),	  std::pow(t2, 3),	  std::pow(t2, 4),
			 0,    1,			 2*t2,  3*std::pow(t2, 2),  4*std::pow(t2, 3);
	return point;
}

Eigen::MatrixXd sixBySixDerivative(int t0, int t1, int t2)
{
	Eigen::MatrixXd point(6, 6);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),	  std::pow(t0, 4),	  std::pow(t0, 5),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),  4*std::pow(t0, 3),  5*std::pow(t0, 4),
			 1,   t1, std::pow(t1, 2),	  std::pow(t1, 3),	  std::pow(t1, 4),	  std::pow(t1, 5),
			 0,    1,			 2*t1,  3*std::pow(t1, 2),  4*std::pow(t1, 3),  5*std::pow(t1, 4),
			 1,   t2, std::pow(t2, 2),	  std::pow(t2, 3),	  std::pow(t2, 4),	  std::pow(t2, 5),
			 0,    1,			 2*t2,  3*std::pow(t2, 2),  4*std::pow(t2, 3),  5*std::pow(t2, 4);
	return point;
}

Eigen::MatrixXd sixBySixDerivative(int t0, int t1)
{
	Eigen::MatrixXd point(6, 6);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),	  std::pow(t0, 4),	  std::pow(t0, 5),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),  4*std::pow(t0, 3),  5*std::pow(t0, 4),
			 0,    0,				2,				 6*t0, 12*std::pow(t0, 2), 20*std::pow(t0, 3),
			 1,   t1, std::pow(t1, 2),	  std::pow(t1, 3),	  std::pow(t1, 4),	  std::pow(t1, 5),
			 0,    1,			 2*t1,  3*std::pow(t1, 2),  4*std::pow(t1, 3),  5*std::pow(t1, 4),
			 0,    0,				2,				 6*t1, 12*std::pow(t1, 2), 20*std::pow(t1, 3);
	return point;
}

Eigen::MatrixXd nineByNineDerivative(int t0, int t1, int t2)
{
	Eigen::MatrixXd point(9, 9);
	point << 1,   t0, std::pow(t0, 2),	  std::pow(t0, 3),	  std::pow(t0, 4),	  std::pow(t0, 5),	  std::pow(t0, 6),	  std::pow(t0, 7),	  std::pow(t0, 8),
			 0,    1,			 2*t0,  3*std::pow(t0, 2),  4*std::pow(t0, 3),  5*std::pow(t0, 4),	6*std::pow(t0, 5),	7*std::pow(t0, 6),	8*std::pow(t0, 7),
			 0,    0,				2,				 6*t0, 12*std::pow(t0, 2), 20*std::pow(t0, 3), 30*std::pow(t0, 4), 42*std::pow(t0, 5), 56*std::pow(t0, 6),
			 1,   t1, std::pow(t1, 2),	  std::pow(t1, 3),	  std::pow(t1, 4),	  std::pow(t1, 5),	  std::pow(t1, 6),	  std::pow(t1, 7),	  std::pow(t1, 8),
			 0,    1,			 2*t1,  3*std::pow(t1, 2),  4*std::pow(t1, 3),  5*std::pow(t1, 4),	6*std::pow(t1, 5),	7*std::pow(t1, 6),	8*std::pow(t1, 7),
			 0,    0,				2,				 6*t1, 12*std::pow(t1, 2), 20*std::pow(t1, 3), 30*std::pow(t1, 4), 42*std::pow(t1, 5), 56*std::pow(t1, 6),
			 1,   t2, std::pow(t2, 2),	  std::pow(t2, 3),	  std::pow(t2, 4),	  std::pow(t2, 5),	  std::pow(t2, 6),	  std::pow(t2, 7),	  std::pow(t2, 8),
			 0,    1,			 2*t2,  3*std::pow(t2, 2),  4*std::pow(t2, 3),  5*std::pow(t2, 4),	6*std::pow(t2, 5),	7*std::pow(t2, 6),	8*std::pow(t2, 7),
			 0,    0,				2,				 6*t2, 12*std::pow(t2, 2), 20*std::pow(t2, 3), 30*std::pow(t2, 4), 42*std::pow(t2, 5), 56*std::pow(t2, 6);
	return point;
}

Eigen::VectorXd calculateAParams(double startTime, double endTime, Eigen::MatrixXd &&pointFinal, int matrixSize, double midTime)
{
	Eigen::MatrixXd pointOne;
	switch (matrixSize) {
		case 4:
			pointOne = fourByFourDerivative(startTime, endTime);
			break;
		case 5:
			pointOne = fiveByFiveDerivative(startTime, midTime, endTime);
			break;
		case 6: {
			if (midTime == 0) {
				pointOne = sixBySixDerivative(startTime, endTime);
			} else {
				pointOne = sixBySixDerivative(startTime, midTime, endTime);
			}
			break;
		case 9:
			pointOne = nineByNineDerivative(startTime, midTime, endTime);
		}
	}
	Eigen::VectorXd pointOneAParams;

	pointOneAParams = pointOne.inverse() * pointFinal;
    ROS_INFO_STREAM("pointAParams = \n" << pointOneAParams);
	return pointOneAParams;
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

void writeToChart(double z, double dz, double d2z, double y, double dy, double d2y, double rz, double drz, double d2rz, double t)
{
	zPose->append(t, z);
	n_zPose = z;
	zSpeed->append(t, dz);
	n_zSpeed = dz;
	zAcc->append(t, d2z);
	n_zAcc = d2z;
	yPose->append(t, y);
	n_yPose = y;
	ySpeed->append(t, dy);
	n_ySpeed = dy;
	yAcc->append(t, d2y);
	n_yAcc = d2y;
	zRotation->append(t, rz);
	n_zRotation = rz;
	zRotationS->append(t, drz);
	n_zRotationS = drz;
	zRotationA->append(t, d2rz);
	n_zRotationA = d2rz;
}

void writeTrajectory(moveit_msgs::RobotTrajectory &trajectory,
					  double startTime,
					  double endTime,
					  std::function<std::vector<double>(double)> motion,
					  const Eigen::MatrixXd &stop)
{
	ROS_INFO_STREAM("Calculation " << startTime << " - " << endTime << " second");
	for (double t = startTime; t <= endTime; t += 0.1) {
		// Vytvorenie prejazdoveho bodu
		trajectory_msgs::JointTrajectoryPoint point;

		// move it to lambda argument
		auto rotation = motion(t);

		// Robot ma 6 klbov
		point.positions.resize(6);
		point.velocities.resize(6);
		point.accelerations.resize(6);

		// Joint 1
		point.positions[0] = rotation[0];

		// Klb 2
		point.positions[1] = rotation[1];

		// Joint 3
		point.positions[2] = rotation[2];

		// Klb 4
		point.positions[3] = rotation[3];

		// Klb 5
		point.positions[4] = rotation[4];

		// Klb 6
		point.positions[5] = rotation[5];

		// Vlozenie casu prejazdu
		point.time_from_start = ros::Duration(t);

		// VLozenie bodu do trajektorie
		trajectory.joint_trajectory.points.push_back(point);
	}
}

std::vector<double> pickSolution(const std::vector<std::vector<double>> &solutions)
{
	std::vector<double> bestSolution;
	double nearest = std::numeric_limits<double>::max();

	for(auto s : solutions) {
		double tmp = 0;
		for (size_t i = 0; i < s.size(); i++) {
			tmp += std::abs(SolutionCalculationHelper[i] - s[i]);
		}

		if (tmp < nearest) {
			nearest = tmp;
			tmp = 0;
			bestSolution = s;
		}

	}
	lastSolution = bestSolution;
	return bestSolution;
}

std::vector<double> solutionFromIkconst(const Eigen::Vector3d &position, double rx, double ry, double rz)
{
	// Vytvorenie modelu z urdf
	robot_model_loader::RobotModelLoader loader("robot_description");

	// Vyber move group a IK algoritmu
	robot_state::JointModelGroup* joint_model_group = loader.getModel()->getJointModelGroup("robot");
	const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();

	// Vytvorenie cielovej polohy x = 1.27, y = 0.0, z = 1.0, rx = 0.0, ry = 1.57, rz = 0.0
	Eigen::Affine3d target = Eigen::Translation3d(position)*
							Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())*
							Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())*
							Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
	xLast = position(0);
	yLast = position(1);
	zLast = position(2);
	rxLast = rx;
	ryLast = ry;
	rzLast = rz;

	// Konverzia z Eigen do geometry_msgs
	geometry_msgs::Pose target_msg;
	tf::poseEigenToMsg(target, target_msg);

	// Premenne vystupujuce vo funkcii getPositionIK
	std::vector<geometry_msgs::Pose> ik_poses = {target_msg};	// Vlozenie cielu do pola
	std::vector<std::vector<double>> solutions;					// Vystup z funkcie s mnozinou rieseni
	std::vector<double> ik_seed_state = {0,0,0,0,0,0};			// Odhadovane riesenie - potrebne iba pre analyticke algoritmy
	kinematics::KinematicsResult result;						// exit_code z funkcie (ci bol vypocet uspesny)

	// Vypocet inverznej kinematiky
	solver->getPositionIK(ik_poses,ik_seed_state, solutions, result, kinematics::KinematicsQueryOptions());

	// Overenie ci bol vypocet uspesny
	if (result.kinematic_error != kinematics::KinematicError::OK) {
		throw std::runtime_error("Unable to compute IK. Error: " + std::to_string(result.kinematic_error));
	}
	return pickSolution(solutions);
}

