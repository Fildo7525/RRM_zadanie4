#include "DerivativeMatrix.hpp"
#include <algorithm>
#include <utility>
#include <vector>

/**
 * Vypocitajte koeficienty a;
 *
 * q = f(a)
 * dq = f(a)
 * 
 * zapis do subory:
 *  q(t) => q(0), q(0.1), ...
 *  dq(t) => dq(0), dq(0.1), ...
 *
 * zapis do ros msg moveit_msg::DisplayTrajectory
 *
 * point.positions[0] = f(a)
 *
 * publisher.publish()
 *
 */

void example(int argc, char **argv);
Eigen::VectorXd calculateAParams(double startTime, double endTime, Eigen::MatrixXd &&pointFinal);
std::vector<double> calculateRobotJoint3Variables(double t, Eigen::VectorXd a);
Eigen::MatrixXd calcultateData(Eigen::MatrixXd a, const double time);
void wiriteTrajectory(moveit_msgs::RobotTrajectory &trajectory,
					  double startTime,
					  double endTime,
					  const Eigen::MatrixXd &jointOne,
					  const Eigen::MatrixXd &jointThree);

std::fstream klby("/home/fildo/Zad4/src/trajectory_visualization/hodnoty.txt", std::ios::out);

int main(int argc, char **argv)
{
	example(argc, argv);
}

void example(int argc, char **argv)
{
	// Vytvorenie node a publishera
	ros::init(argc, argv, "trajectory_visualization");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);

	// Sprava pre trajektoriu
	moveit_msgs::RobotTrajectory trajectory;
	// Mena klbov musia byt vyplnene
	trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

	ROS_INFO_STREAM("Joint one calc");
	Eigen::MatrixXd j1(6,1);
	j1 << 0, 0, 0, M_PI/2, 0, 0;
	ROS_INFO_STREAM("Base data set");
	Eigen::VectorXd jointOne = calculateAParams(T0, T2, std::move(j1));

	ROS_INFO_STREAM("Joint two calc pt1");
	Eigen::MatrixXd j3pt1(6,1);
	j3pt1 << 0, 0, 0, M_PI/3, 0, 0;
	Eigen::VectorXd jointThree = calculateAParams(T0, T1, std::move(j3pt1));



	ROS_INFO_STREAM("Calculation 0 - 1 second");
	// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
	wiriteTrajectory(trajectory, T0, T1, jointOne, jointThree);



	ROS_INFO_STREAM("Calculation joint two pt 2");
	Eigen::MatrixXd j3pt2(6,1);
	j3pt2 << M_PI/3, 0, 0, 0, 0, 0;
	jointThree = calculateAParams(T1, T2, std::move(j3pt2));



	ROS_INFO_STREAM("Calculation 1 - 4 second");
	wiriteTrajectory(trajectory, T1, T2, jointOne, jointThree);



	// Sprava pre vizualizaciu
	moveit_msgs::DisplayTrajectory display_trajectory;

	// Potrebne vyplnit nazov modelu
	display_trajectory.model_id = "abb_irb";

	// Vlozenie vypocitanej trajektorie
	display_trajectory.trajectory.push_back(trajectory);

	// Publikuj trajektoriu kazde 2 sekundy
	ros::Rate loop_rate(0.5);
	while (ros::ok()) {
		publisher.publish(display_trajectory);
		loop_rate.sleep();
	}
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
	Eigen::MatrixXd data(4,1);
	double entry = 0;

	// Position
	for (size_t i = 0; i < a.size(); i++) {
		entry += a(i) * std::pow(time, i);
	}
	data(0) = entry;

	// Velocity
	entry = 0;
	for (size_t i = 1; i < a.size(); i++) {
		entry += i*a(i) * std::pow(time, i-1);
	}
	data(1) = entry;

	// Acceleration
	entry = 0;
	for (size_t i = 2; i < a.size(); i++) {
		entry += i*(i-1)*a(i) * std::pow(time, i-2);
	}
	data(2) = entry;

	// Yank
	entry = 0;
	for (size_t i = 3; i < a.size(); i++) {
		entry += i*(i-1)*(i-2)*a(i) * std::pow(time, i-3);
	}
	data(3) = entry;

	return data;
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
		klby << jointOneData(0) << '\n';
		klby << jointOneData(1) << '\n';
		klby << jointOneData(2) << '\n';
		klby << jointOneData(3) << '\n';


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
		klby << jointThreeData(0) << '\n';
		klby << jointThreeData(1) << '\n';
		klby << jointThreeData(2) << '\n';
		klby << jointThreeData(3) << '\n';

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

