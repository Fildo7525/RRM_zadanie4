#include "DerivativeMatrix.hpp"
#include <algorithm>
#include <qchart.h>
#include <qchartview.h>
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
	zPose->setName("position");
	zSpeed->setName("speed");
	zAcc->setName("acceleration");
	yPose->setName("yank");
	ySpeed->setName("position");
	yAcc->setName("speed");
	acceleration2->setName("acceleration");
	yank2->setName("yank");

	// Vytvorenie node a publishera
	ros::init(argc, argv, "trajectory_visualization");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);

	// Sprava pre trajektoriu
	moveit_msgs::RobotTrajectory trajectory;
	// Mena klbov musia byt vyplnene
	trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

	{
		ROS_INFO_STREAM("Move down by z");
		Eigen::MatrixXd conditions(4,1);
		conditions << 1.6, 0, 1, 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T0, T1, std::move(conditions), 4);

		ROS_INFO_STREAM("Calculation 0 - 1 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		wiriteTrajectory(trajectory, T0, T1, firstMotion, conditions, 1);
	}

	{
		ROS_INFO_STREAM("Rotate along z");
		Eigen::MatrixXd conditions(4,1);
		conditions << 0, 0, M_PI/2., 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T1, T2, std::move(conditions), 4);

		ROS_INFO_STREAM("Calculation 0 - 1 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		wiriteTrajectory(trajectory, T1, T2, firstMotion, conditions, 1);
	}

	{
		ROS_INFO_STREAM("Rotate along z");
		Eigen::MatrixXd conditions(4,1);
		conditions << 0, 0, M_PI/2., 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T2, T3, std::move(conditions), 4);

		ROS_INFO_STREAM("Calculation 0 - 1 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		wiriteTrajectory(trajectory, T2, T3, firstMotion, conditions, 1);
	}

	{
		ROS_INFO_STREAM("Move down by z");
		Eigen::MatrixXd conditions(4,1);
		conditions << 0, 0, 0.5, 0.5, 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion = calculateAParams(T3, T5, std::move(conditions), 4, T4);

		Eigen::MatrixXd conditions3(4,1);
		conditions << 1, 0, 1, 1.6, 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion2 = calculateAParams(T3, T5, std::move(conditions2), 4, T4);

		Eigen::MatrixXd conditions2(4,1);
		conditions << M_PI/2., 0, M_PI/2., 0, 0;
		ROS_INFO_STREAM("Base data set");
		Eigen::VectorXd firstMotion2 = calculateAParams(T3, T5, std::move(conditions2), 4, T4);

		ROS_INFO_STREAM("Calculation 0 - 1 second");
		// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
		wiriteTrajectory(trajectory, T3, T4, firstMotion, conditions, 1);
	}


	ROS_INFO_STREAM("Move down by z");
	Eigen::MatrixXd firstStop(4,1);
	firstStop << 1.6, 0, 1, 0;
	ROS_INFO_STREAM("Base data set");
	Eigen::VectorXd firstMotion = calculateAParams(T0, T1, std::move(firstStop), 4);

	ROS_INFO_STREAM("Calculation 0 - 1 second");
	// V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
	wiriteTrajectory(trajectory, T0, T1, firstMotion, firstStop, 1);





	ROS_INFO_STREAM("Rotate around z");
	Eigen::MatrixXd j3pt1(4,1);
	j3pt1 << 0, 0, M_PI/6, 0;
	Eigen::VectorXd jointThree = calculateAParams(T1, T2, std::move(j3pt1), 4);






	ROS_INFO_STREAM("Calculation joint two pt 2");
	Eigen::MatrixXd j3pt2(6,1);
	j3pt2 << M_PI/6, 0, 0, 0, 0, 0;
	jointThree = calculateAParams(T1, T2, std::move(j3pt2), 4);



	ROS_INFO_STREAM("Calculation 1 - 4 second");
	wiriteTrajectory(trajectory, T1, T2, firstMotion, jointThree);



	// Sprava pre vizualizaciu
	moveit_msgs::DisplayTrajectory display_trajectory;

	// Potrebne vyplnit nazov modelu
	display_trajectory.model_id = "abb_irb";

	QtCharts::QChart *firstJoint = new QtCharts::QChart();
	firstJoint->addSeries(zPose);
	firstJoint->addSeries(zSpeed);
	firstJoint->addSeries(zAcc);
	firstJoint->addSeries(yPose);
	firstJoint->setTitle("Joint One position and its derivatives");
	firstJoint->createDefaultAxes();
	firstJoint->axes(Qt::Horizontal).first()->setRange(0, 4);
	QtCharts::QChartView *chartView1 = new QtCharts::QChartView(firstJoint);

	QtCharts::QChart *thirdJoint = new QtCharts::QChart();
	thirdJoint->addSeries(ySpeed);
	thirdJoint->addSeries(yAcc);
	thirdJoint->addSeries(acceleration2);
	thirdJoint->addSeries(yank2);
	thirdJoint->setTitle("Joint Three position and its derivatives");
	thirdJoint->createDefaultAxes();
	thirdJoint->axes(Qt::Horizontal).first()->setRange(0, 4);
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


