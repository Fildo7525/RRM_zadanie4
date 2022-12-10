#include "DerivativeMatrix.hpp"
#include <algorithm>
#include <qchart.h>
#include <qchartview.h>
#include <qnamespace.h>
#include <utility>
#include <vector>
#include <iterator>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>

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
	QApplication a(argc, argv);
	zPose->setName("z_position");
	zSpeed->setName("z_speed");
	zAcc->setName("z_acceleration");
	yPose->setName("y_pose");
	ySpeed->setName("y_speed");
	yAcc->setName("y_acceleration");
	zRotation->setName("z_rotation");

	// Vytvorenie node a publishera
	ros::init(argc, argv, "trajectory_visualization");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);

	// Sprava pre trajektoriu
	moveit_msgs::RobotTrajectory trajectory;
	// Mena klbov musia byt vyplnene
	trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

		// auto data = calcultateData(motion, t);
		// writeToChart(zyr, data, t);
		//
		// std::vector<double> rotation;
		// if (zyr == 1) {
		// 	rotation = solutionFromIkconst({1, 0, data(0)}, 0, M_PI/2., 0);
		// } else if (zyr == 2) {
		// 	rotation = solutionFromIkconst({1, data(0), 1}, 0, M_PI/2., 0);
		// } else if (zyr == 3) {
		// 	rotation = solutionFromIkconst({1, 0, 1}, 0, M_PI/2., data(0));
		// }
	{
		ROS_INFO_STREAM("Move down by z");
		Eigen::MatrixXd conditions(4,1);
		conditions << 1.6, 0, 1, 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T0, T1, std::move(conditions), 4);

		ROS_INFO_STREAM("Calculation 0 - 1 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		writeTrajectory(trajectory, T0, T1, [&] (double t) {
					   auto data = calcultateData(firstMotion, t);
					   zPose->append(t, data(0));
					   zSpeed->append(t, data(1));
					   zAcc->append(t, data(2));
					   yPose->append(t, yLast);
					   writeToChart(data(0), data(1), data(2), yLast, n_ySpeed, n_yAcc, rzLast, t);
					   return solutionFromIkconst({1, 0, data(0)}, 0, M_PI/2., 0);
				   }, conditions);
	}

	{
		ROS_INFO_STREAM("Rotate along z");
		Eigen::MatrixXd conditions(4,1);
		conditions << 0, 0, M_PI/2., 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T1, T2, std::move(conditions), 4);

		ROS_INFO_STREAM("Calculation 1 - 2 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		writeTrajectory(trajectory, T1, T2, [&] (double t) {
					   auto data = calcultateData(firstMotion, t);
					   writeToChart(n_zPose, n_zSpeed, n_zAcc, yLast, n_ySpeed, n_yAcc, data(0), t);
					   return solutionFromIkconst({1, 0, zLast}, 0, M_PI/2., data(0));
				   }, conditions);
	}

	{
		ROS_INFO_STREAM("wate for one second");
		ROS_INFO_STREAM("Calculation 2 - 3 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		writeTrajectory(trajectory, T2, T3,
				   		[&] (double t) {
							writeToChart(n_zPose, n_zSpeed, n_zAcc, yLast, n_ySpeed, n_yAcc, n_zRotation, t);
					  		return lastSolution;
					    },Eigen::MatrixXd());
	}

	{
		ROS_INFO_STREAM("Move left by y and up alongside z and rotate in rz");
		Eigen::MatrixXd conditions(5,1);
		conditions << 0, 0, 0.5, 0.5, 0;
		ROS_INFO_STREAM("Calculation for y in  3 - 5 second");
		Eigen::VectorXd y = calculateAParams(T3, T5, std::move(conditions), 5, T4);

		Eigen::MatrixXd conditions2(5,1);
		conditions2 << 1, 0, 1, 1.6, 0;
		ROS_INFO_STREAM("Calculation for z in  3 - 5 second");
		Eigen::VectorXd z = calculateAParams(T3, T5, std::move(conditions2), 5, T4);

		Eigen::MatrixXd conditions3(6,1);
		conditions3 << M_PI/2., 0, M_PI/2., 0, 0, 0;
		ROS_INFO_STREAM("Calculation for rz in  3 - 5 second");
		Eigen::VectorXd rz = calculateAParams(T3, T5, std::move(conditions3), 6, T4);

		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		writeTrajectory(trajectory, T3, T5, [&] (double t) {
							auto data = calcultateData(y, t);
							auto data1 = calcultateData(z, t);
							auto data2 = calcultateData(rz, t);
							writeToChart(data1(0), data1(1), data1(2), data(0), data(1), data(2), data2(0), t);
							return solutionFromIkconst({1, data(0), data1(0)}, 0, M_PI/2., data2(0));
				   }, conditions);
	}

	{
		ROS_INFO_STREAM("Move to right alongside y");
		Eigen::MatrixXd conditions(4,1);
		conditions << 0.5, 0, 0, 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T5, T6, std::move(conditions), 4);

		ROS_INFO_STREAM("Calculation 5 - 9 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		writeTrajectory(trajectory, T5, T6,
				   		[&] (double t) {
							auto data = calcultateData(firstMotion, t);
							writeToChart(n_zPose, n_zSpeed, n_zAcc, data(0), data(1), data(2), n_zRotation, t);
							return solutionFromIkconst({1, data(0), zLast}, 0, M_PI/2., rzLast);
					    },
					    conditions);
	}


	// Sprava pre vizualizaciu
	moveit_msgs::DisplayTrajectory display_trajectory;

	// Potrebne vyplnit nazov modelu
	display_trajectory.model_id = "abb_irb";

	QtCharts::QChart *firstJoint = new QtCharts::QChart();
	firstJoint->addSeries(zPose);
	firstJoint->addSeries(zSpeed);
	firstJoint->addSeries(zAcc);
	firstJoint->setTitle("z axis");
	firstJoint->createDefaultAxes();
	firstJoint->axes(Qt::Horizontal).first()->setRange(0, 9);
	firstJoint->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
	firstJoint->axes(Qt::Vertical).first()->setTitleText("Position (rad), Velocity (rad/s), Acceleration (rad/s^2)");
	QtCharts::QChartView *chartView1 = new QtCharts::QChartView(firstJoint);

	QtCharts::QChart *thirdJoint = new QtCharts::QChart();
	thirdJoint->addSeries(ySpeed);
	thirdJoint->addSeries(yAcc);
	thirdJoint->addSeries(yPose);
	thirdJoint->addSeries(zRotation);
	thirdJoint->setTitle("y axis and z rotation");
	thirdJoint->createDefaultAxes();
	thirdJoint->axes(Qt::Horizontal).first()->setRange(0, 9);
	thirdJoint->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
	thirdJoint->axes(Qt::Vertical).first()->setTitleText("Position (rad), Velocity (rad/s), Acceleration (rad/s^2)");
	QtCharts::QChartView *chartView = new QtCharts::QChartView(thirdJoint);

	QMainWindow w;
	w.setCentralWidget(chartView1);
	w.resize(1000, 900);
	w.show();

	QMainWindow w2;
	w2.setCentralWidget(chartView);
	w2.resize(1000, 900);
	w2.show();

	a.exec();

	// Vlozenie vypocitanej trajektorie
	display_trajectory.trajectory.push_back(trajectory);

	// Publikuj trajektoriu kazde 2 sekundy
	ros::Rate loop_rate(0.5);
	while (ros::ok()) {
		publisher.publish(display_trajectory);
		loop_rate.sleep();
	}
}


