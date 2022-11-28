#include <ros/ros.h>
#include <Eigen/Geometry>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_example");

    // Inicializacia vektora po prvkoch
    Eigen::VectorXd v1(3);
    v1 << 1, 2, 3;
    ROS_INFO_STREAM("v1 = \n" << v1);

    // Iniclizacia nuloveho vektora
    Eigen::VectorXd v2 = Eigen::VectorXd::Zero(3);
    ROS_INFO_STREAM("v2 = \n" << v2);

    // Inicializacia 4x4 matice po prvkoch
    Eigen::MatrixXd m1(3,3);
    m1 <<  1, 2, 3,
          4, 5, 6,
          7, 8, 9;
    ROS_INFO_STREAM("m1:\n" << m1);

    // Vytvorenie identickej matice
    Eigen::MatrixXd m2 = Eigen::MatrixXd::Identity(3,3);
    ROS_INFO_STREAM("m2 = \n" << m2);

    // Vytvorenie nulovej matice
    Eigen::MatrixXd m3 = Eigen::MatrixXd::Zero(3,3);
    ROS_INFO_STREAM("m2 = \n" << m3);

    // sucet
    ROS_INFO_STREAM("m1 + m2 = \n" << (m1 + m2));

    // nasobenie
    ROS_INFO_STREAM("m1 * v1 = \n" << (m1 * v1));

    // inverzia matice
    ROS_INFO_STREAM("m1^-1 = \n" << (m1.inverse()));

    // Skalarny sucin
    ROS_INFO_STREAM("v1 . v2 = \n" << v1.dot(v2));

    // Vektorovy sucin
    Eigen::Vector3d x1 = {1,2,3};
    Eigen::Vector3d x2 = {1,0,0};
    ROS_INFO_STREAM(" [1,2,3] x [1,0,0] = \n" << x1.cross(x2));
    return 0;
}