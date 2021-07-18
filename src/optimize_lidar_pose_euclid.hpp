#pragma once

#include<Eigen/Geometry> // needed for inverse

namespace ittik {

  template<typename error_functionT>
  auto gradient_descent_step(const Eigen::Matrix<double, 6, 1>& estimate, error_functionT&& error_function)
  {
    const double epsilon = 1e-7;
    Eigen::Matrix<double, 6, 2> jacobian;
    auto current_error = error_function(estimate);
    for(int dim = 0; dim < 6; ++dim)
    {
      Eigen::Matrix<double, 6, 1> delta;
      delta.setZero();
      delta(dim) = epsilon;
      auto estimate_plus = estimate + delta;
      jacobian.row(dim) = (error_function(estimate_plus) - current_error) / epsilon;
    }

    //const double step_size = 1e-5;
    //auto delta = (step_size * jacobian * current_error).eval();
    //return estimate - delta;
    ////gaus newton
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd((jacobian * jacobian.transpose()).eval(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    //auto delta = svd.solve(jacobian * current_error).eval();
    //return estimate - delta;

    //levenberg_marquardt
    double lambda = 1e-9; // para
    auto prev_error = current_error;
    Eigen::Matrix<double, 6, 1> delta;
    auto estimate_updated = estimate;
    do
    {
      auto jacjacT = (jacobian * jacobian.transpose()).eval();
      Eigen::JacobiSVD<Eigen::MatrixXd> svd((jacjacT + lambda * decltype(jacjacT)::Identity()).eval(), Eigen::ComputeThinU | Eigen::ComputeThinV);
      delta = svd.solve(jacobian * prev_error).eval();

      auto estimate_prev = estimate_updated;
      estimate_updated = estimate_prev - delta;
      auto error = error_function(estimate_updated);
      if(error.norm() < prev_error.norm())
      {
        lambda /= 2;
        prev_error = error;
      }
      else
      {
        lambda *= 2;
        estimate_updated = estimate_prev;
      }
    } while(delta.norm() > 1e-8);
    return estimate_updated;
  }
}

