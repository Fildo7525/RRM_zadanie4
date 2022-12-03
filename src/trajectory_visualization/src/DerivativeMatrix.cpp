#include "DerivativeMatrix.hpp"

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

