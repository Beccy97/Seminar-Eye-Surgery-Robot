#pragma once
#include <vector>
#include <Eigen/Eigen>


Eigen::Matrix<float,5,1> Jacobian_pseudoinverse(Eigen::Matrix<float,5,1> current_Position, Eigen::Matrix<float,6,1> step);
