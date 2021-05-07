#pragma once

#include<Eigen/Geometry> // needed for inverse

namespace ittik {

  template<typename error_functionT>
  auto gradient_descent_step(const liespline::Isometryd3& estimate, error_functionT&& error_function)
  {
    const double epsilon = 1e-7;
    Eigen::Matrix<double, 6, 1> jacobian;
    auto current_error = error_function(estimate);
    for(int dim = 0; dim < 6; ++dim)
    {
      Eigen::Matrix<double, 6, 1> delta;
      delta.setZero();
      delta(dim) = epsilon;
      auto estimate_plus = estimate * liespline::expse3(delta);
      jacobian.row(dim) = (error_function(estimate_plus) - current_error) / epsilon;
    }

    const double step_size = 1e-7;
    auto delta = (step_size * jacobian * current_error).eval();
    ////gaus newton
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd((jacobian * jacobian.transpose()).eval(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    //auto delta = svd.solve(jacobian * current_error).eval();
    return estimate * liespline::expse3(-delta);
  }
}

