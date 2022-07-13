#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/rnea.hpp>

#include <iostream>

int main()
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);
  pinocchio::Data data(model);

  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  const std::string urdf_filename = "/opt/pal/ferrum/share/tiago_description/robots/tiago2.urdf";
  pinocchio::Model pin_model;
  int input;
  std::cout << "input something to build Model with pinocchio:" << std::endl;
  std::cin >> input ;
  std::cout << "input: " << input << std::endl;
  pinocchio::urdf::buildModel(urdf_filename,pin_model);
  std::cout << "pinocchio model name: " << pin_model.name << std::endl;

  const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
  std::cout << "tau = " << tau.transpose() << std::endl;
}
