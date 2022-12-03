#include "DerivativeMatrix.hpp"
#include <algorithm>
#include <utility>
#include <vector>
#include <iterator>

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


int main(int argc, char **argv)
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

	for(auto var : data) {
		std::copy(var.cbegin(), var.cend(), std::ostream_iterator<double>(klby, "\n"));
		klby << '\n';
	}
	klby.close();
	// Vlozenie vypocitanej trajektorie
	display_trajectory.trajectory.push_back(trajectory);

	// Publikuj trajektoriu kazde 2 sekundy
	ros::Rate loop_rate(0.5);
	while (ros::ok()) {
		publisher.publish(display_trajectory);
		loop_rate.sleep();
	}
}


