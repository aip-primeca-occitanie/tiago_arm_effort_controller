#include <XmlRpc.h>

#include "controller.hpp"
#include "ros_color_stream.h"

#define URDF_PATH "/opt/pal/ferrum/share/tiago_description/robots/tiago2.urdf"

template <typename T>
bool is_in_vector(const std::vector<T> &vector, const T &elt)
{
  return vector.end() != std::find(vector.begin(), vector.end(), elt);
}

namespace force_control
{
  bool MyTiagoController::initRequest(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle &root_nh,
                                      ros::NodeHandle &controller_nh,
                                      ClaimedResources &claimed_resources)
  {
    ROS_O(">>> Initialising MyTiagoController");
    // Check if construction finished cleanly
    if (state_ != CONSTRUCTED)
    {
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // Get a pointer to the joint effort control interface
    hardware_interface::EffortJointInterface *effort_iface =
        robot_hw->get<hardware_interface::EffortJointInterface>();

    if (!effort_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type EffortJointInterface."
                " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
    }

    // Get a pointer to the joint position control interface
    hardware_interface::JointStateInterface *joint_state_iface =
        robot_hw->get<hardware_interface::JointStateInterface>();
    if (!joint_state_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type JointStateInterface."
                " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
    }

    // Clear resources associated at both interfaces
    effort_iface->clearClaims();
    joint_state_iface->clearClaims();

    if (!init(effort_iface, joint_state_iface, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }

    // Saves the resources claimed by this controller
    claimed_resources.push_back(hardware_interface::InterfaceResources(
        hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>(),
        effort_iface->getClaims()));
    effort_iface->clearClaims();

    // Changes state to INITIALIZED
    state_ = INITIALIZED;
    return true;
  }

  bool MyTiagoController::init(hardware_interface::EffortJointInterface *effort_iface,
                               hardware_interface::JointStateInterface *joint_state_iface,
                               ros::NodeHandle & /*root_nh*/, ros::NodeHandle &control_nh)
  {
    ROS_O(">>> Loading MyTiagoController");

    // Load the urdf model
    const std::string urdf_filename = URDF_PATH;

    ROS_Y(">>> creating the pinocchio model");
    pinocchio::urdf::buildModel(urdf_filename, pin_model_);
    pinocchio::Data pin_data_(pin_model_);

    ROS_Y(">>> parsing the joints to keep in the reduced model");

    std::vector<std::string> list_of_joints_to_keep_unlocked_by_name;
    list_of_joints_to_keep_unlocked_by_name.push_back("torso_lift_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_1_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_2_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_3_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_4_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_5_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_6_joint");
    list_of_joints_to_keep_unlocked_by_name.push_back("arm_7_joint");

    std::vector<pinocchio::JointIndex> list_of_joints_to_keep_unlocked_by_id;
    Eigen::VectorXd q_rand = pinocchio::randomConfiguration(pin_model_);
    for (std::vector<std::string>::const_iterator it = list_of_joints_to_keep_unlocked_by_name.begin();
         it != list_of_joints_to_keep_unlocked_by_name.end(); ++it)
    {
      const std::string &joint_name = *it;
      if (pin_model_.existJointName(joint_name))
        list_of_joints_to_keep_unlocked_by_id.push_back(pin_model_.getJointId(joint_name));
      else
        std::cout << "joint: " << joint_name << " does not belong to the model";
    }
    // Transform the list into a list of joints to lock
    std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
    for (pinocchio::JointIndex joint_id = 1; joint_id < pin_model_.joints.size(); ++joint_id)
    {
      const std::string joint_name = pin_model_.names[joint_id];
      if (is_in_vector(list_of_joints_to_keep_unlocked_by_name, joint_name))
        continue;
      else
      {
        list_of_joints_to_lock_by_id.push_back(joint_id);
      }
    }

    ROS_Y(">>> creating the reduced model");
    // Build the reduced model from the list of lock joints
    reduced_model_ = pinocchio::buildReducedModel(pin_model_, list_of_joints_to_lock_by_id, q_rand);
    pinocchio::Data reduced_data_(reduced_model_);

    // Display the parameters of the reduced model for the TIAGo's arm
    ROS_C("joints (name: id): " << reduced_model_.njoints);
    for (size_t i = 0; i < reduced_model_.njoints; i++)
      ROS_G(reduced_model_.names[i] << ": " << reduced_model_.joints[i].id());
    ROS_C("lowerPositionLimit: " << reduced_model_.lowerPositionLimit.size());
    for (size_t i = 0; i < reduced_model_.lowerPositionLimit.size(); i++)
      ROS_G(reduced_model_.lowerPositionLimit[i]);
    ROS_C("upperPositionLimit: " << reduced_model_.upperPositionLimit.size());
    for (size_t i = 0; i < reduced_model_.upperPositionLimit.size(); i++)
      ROS_G(reduced_model_.upperPositionLimit[i]);
    ROS_C("velocityLimit: " << reduced_model_.velocityLimit.size());
    for (size_t i = 0; i < reduced_model_.velocityLimit.size(); i++)
      ROS_G(reduced_model_.velocityLimit[i]);
    ROS_C("damping: " << reduced_model_.damping.size());
    for (size_t i = 0; i < reduced_model_.damping.size(); i++)
      ROS_G(reduced_model_.damping[i]);
    ROS_C("friction: " << reduced_model_.friction.size());
    for (size_t i = 0; i < reduced_model_.friction.size(); i++)
      ROS_G(reduced_model_.friction[i]);
    ROS_C("effortLimit: " << reduced_model_.effortLimit.size());
    for (size_t i = 0; i < reduced_model_.effortLimit.size(); i++)
      ROS_G(reduced_model_.effortLimit[i]);

    for (size_t i = 0; i < joint_names_.size(); i++)
    {
      // Checks joint type from param server
      std::string control_type;
      if (!control_nh.getParam("joints/" + joint_names_[i] + "/type", control_type))
      {
        ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " interface type");
        return false;
      }

      if (control_type == "actuated" ||
          control_type == "no_control") // If joint is actuated or constantly commanded to zero
      {
        // Read the actuator parameters from param server
        ActuatorParameters actuator_parameters;
        if (!control_nh.getParam("joints/" + joint_names_[i] + "/motor_torque_constant",
                                 actuator_parameters.motor_torque_constant))
        {
          ROS_ERROR_STREAM("Could not find motor torque constant for joint " << joint_names_[i]);
          return false;
        }
        if (!control_nh.getParam("joints/" + joint_names_[i] + "/reduction_ratio",
                                 actuator_parameters.reduction_ratio))
        {
          ROS_ERROR_STREAM("Could not find reduction ratio for joint " << joint_names_[i]);
          return false;
        }

        // Reads the optional gravity compensation parameters
        GravityCompensationParameters friction_parameters;
        if (!control_nh.getParam("viscous_friction", friction_parameters.viscous_friction))
          ROS_WARN_STREAM("No viscous friction defined for joint "
                          << joint_names_[i] << ". Setting it to 0.0");
        if (!control_nh.getParam("velocity_tolerance", friction_parameters.velocity_tolerance))
          ROS_WARN_STREAM("No velocity tolerance defined for joint "
                          << joint_names_[i] << ". Setting it to 0.0");
        if (!control_nh.getParam("static_friction", friction_parameters.static_friction))
          ROS_WARN_STREAM("No static friction defined for joint " << joint_names_[i]
                                                                  << ". Setting it to 0.0");

        try
        {
          // Try to get an effort interface handle to command the joint in effort
          hardware_interface::JointHandle joint_handle =
              effort_iface->getHandle(joint_names_[i]);
          // Creates an actuated joint and insert in the map of actuated joints
          ActuatedJoint actuated_joint;
          actuated_joint.joint_handle = joint_handle;
          actuated_joint.actuator_parameters = actuator_parameters;
          actuated_joint.friction_parameters = friction_parameters;
          actuated_joints_.insert(std::make_pair(joint_names_[i], actuated_joint));
        }
        catch (...)
        {
          ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Effort interface");
          return false;
        }
        // Insert the joint in the map of joint types according to his type
        if (control_type == "actuated")
          joint_types_.insert(std::make_pair(joint_names_[i], JointType::ACTUATED));
        else if (control_type == "no_control")
          joint_types_.insert(std::make_pair(joint_names_[i], JointType::ACTUATED_NO_CONTROL));
      }
      else // If static joint
      {
        try
        {
          // Try to get a joint state handle which only allows us to read the current states
          // of the joint
          hardware_interface::JointStateHandle joint_state_handle =
              joint_state_iface->getHandle(joint_names_[i]);
          // Insert this handle in the map of static joints
          static_joints_.insert(std::make_pair(joint_names_[i], joint_state_handle));
        }
        catch (...)
        {
          ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Position interface");
          return false;
        }
        // Insert the joint in the map of joint types
        joint_types_.insert(std::make_pair(joint_names_[i], JointType::STATIC));
      }
    }

    assert(joint_types_.size() == joint_names_.size());
    assert(joint_types_.size() == actuated_joints_.size() + static_joints_.size());

    // Iinitializa q_act_, q_zero_, tau_cmd_
    q_act_.resize(joint_names_.size());
    q_zero_.resize(joint_names_.size());
    tau_cmd_.resize(joint_names_.size());

    q_act_.setZero();
    q_zero_.setZero();
    tau_cmd_.setZero();

    // Allows to modify the gravity compensation parameters from the rqt reconfigure
    for (auto it = actuated_joints_.begin(); it != actuated_joints_.end(); it++)
    {
      ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(control_nh));
      ddr_->RegisterVariable(&it->second.friction_parameters.viscous_friction,
                             it->first + "/viscous_friction_gain");
      ddr_->RegisterVariable(&it->second.friction_parameters.static_friction,
                             it->first + "/static_friction_gain");
      ddr_->RegisterVariable(&it->second.friction_parameters.velocity_tolerance,
                             it->first + "/velocity_tolerance_gain");
      ddr_->PublishServicesTopics();
    }

    return true;
  }

  /**
   * @brief
   *
   * @param Xd
   * @param X
   * @param dXd
   * @param dX
   * @param ddXd
   * @param ddXn
   * @param J
   * @param A
   * @param H
   * @return Eigen::VectorXd
   */
  Eigen::VectorXd MyTiagoController::computedTorqueController(
      Eigen::VectorXd Xd, Eigen::VectorXd X,    Eigen::VectorXd dXd,
      Eigen::VectorXd dX, Eigen::VectorXd ddXd, Eigen::VectorXd ddXn,
      Eigen::MatrixXd J,  Eigen::MatrixXd A,    Eigen::MatrixXd H)
  {
    double Kp = 1;
    double Kd = 2 * sqrt(Kp);
    auto ex = Xd - X;
    auto edx = dXd - dX;
    auto Jp = J.inverse();
    auto W = Kp * ex + Kd * edx + ddXd - ddXn;
    auto jpw = Jp * W;
    auto tau = A * jpw + H;
    return tau;
  }

  void MyTiagoController::update(const ros::Time &time, const ros::Duration &period)
  {
    //  Read the current position of all the joints
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (joint_types_[joint_names_[i]] == JointType::ACTUATED ||
          joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL)
        q_act_[i] = actuated_joints_[joint_names_[i]].joint_handle.getPosition();
      else
        q_act_[i] = static_joints_[joint_names_[i]].getPosition();
    }

    // Calculate the minimum torque to maintain the robot at the current position
    tau_cmd_.setZero();
    Eigen::VectorXd q = pinocchio::neutral(reduced_model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(reduced_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(reduced_model_.nv);
    pinocchio::Data data(reduced_model_); // !!!!!!!! WHY THIS WORKS AND NOT REDUCED_DATA_ ? !!!!!! --> IS IT BECAUSE IT NEEDS TO BE IN THE UPDATE ?
    const Eigen::VectorXd &tau = pinocchio::rnea(reduced_model_, data, q_act_, q_zero_, q_zero_);
    ROS_Y("TAU: " << tau.transpose());

    // For all the joints...
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // ...check those one that are actuated
      if (joint_types_[joint_names_[i]] == JointType::ACTUATED)
      {
        // Translate the calculated torque to desired effort by integrating the frictions of
        // the motor + the ger ration + motor constants
        ActuatedJoint &actuated_joint = actuated_joints_[joint_names_[i]];
        double desired_torque = tau(i);
        double actual_velocity = actuated_joint.joint_handle.getVelocity();

        // ROS_RED_STREAM("joint name: " << actuated_joint.joint_handle.getName()); //TODO
        // ROS_GREEN_STREAM("motor_torque_constant: " << actuated_joint.actuator_parameters.motor_torque_constant);
        // ROS_GREEN_STREAM("reduction_ratio: " << actuated_joint.actuator_parameters.reduction_ratio);
        // ROS_GREEN_STREAM("viscous_friction: " << actuated_joint.friction_parameters.viscous_friction);
        // ROS_GREEN_STREAM("static_friction: " << actuated_joint.friction_parameters.static_friction);
        // ROS_GREEN_STREAM("velocity_tolerance: " << actuated_joint.friction_parameters.velocity_tolerance);

        desired_torque += actuated_joint.friction_parameters.viscous_friction * actual_velocity;
        if (actual_velocity > actuated_joint.friction_parameters.velocity_tolerance)
          desired_torque += actuated_joint.friction_parameters.static_friction;
        else
          desired_torque -= actuated_joint.friction_parameters.static_friction;

        double desired_effort =
            desired_torque / (actuated_joint.actuator_parameters.motor_torque_constant *
                              actuated_joint.actuator_parameters.reduction_ratio);

        if (std::isnan(desired_effort)) // If desired effort is not valid
        {
          ROS_ERROR_STREAM("Desired effort is not valid for joint "
                           << joint_names_[i] << " = " << desired_effort);
          return;
        }
        else
        {
          // Command an effort to the joint via ros_cotrol interface
          ROS_O("COMMAND LOOP");

          double secs = ros::Time::now().toSec();
          double Fc = 0.15;
          double temp = 0.3 * sin(2 * 3.141592653 * Fc * secs);
          double temp2 = 0.3 * (2 * 3.141592653 * Fc) * cos(2 * 3.141592653 * Fc * secs);
          double temp3 = 0.3 * (2 * 3.141592653 * Fc) * (2 * 3.141592653 * Fc) * -sin(2 * 3.141592653 * Fc * secs);
          Eigen::VectorXd Xd(6);
          Xd << temp,0.,0.,0.,0.,0.;
          Eigen::VectorXd dXd(6);
          dXd << temp2,0.,0.,0.,0.,0.;
          Eigen::VectorXd ddXd(6);
          ddXd << temp3,0.,0.,0.,0.,0.;
          pinocchio::Data::Matrix6x J(6,pin_model_.nv);
          J.setZero();
          pinocchio::computeJointJacobian(pin_model_,data,q,7,J);
          const auto &A = pinocchio::crba(pin_model_,data,q);
          const Eigen::VectorXd &H = pinocchio::rnea(reduced_model_, data, q_act_, q_zero_, q_zero_);

          computedTorqueController(Xd, Xd, dXd, dXd, ddXd, ddXd, J, A, H);
          ROS_Y("TAU[" << i << "]: " << tau.transpose());

          if (i == 1)
          {
            // ROS_GREEN_STREAM("desired_torque: " << desired_torque);
            // ROS_GREEN_STREAM("actuated_joint.actuator_parameters.motor_torque_constant: " << actuated_joint.actuator_parameters.motor_torque_constant);
            // ROS_GREEN_STREAM("actuated_joint.actuator_parameters.reduction_ratio: " << actuated_joint.actuator_parameters.reduction_ratio);
            // ROS_CYAN_STREAM("desired effort: " << desired_effort);

            // double secs = ros::Time::now().toSec();
            // double Fc = 0.15;
            // desired_effort = 0.3 * sin(2 * 3.141592653 * Fc * secs);

            // std_msgs::Float64 msg;
            // msg.data = desired_effort;

            // ROS_BLUE_STREAM("my modified effort: " << desired_effort);
            // mypublisher_.publish(msg);
          }

          actuated_joint.joint_handle.setCommand(desired_effort);
        }
      }
      else if (joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL)
      {
        // ...and those one that are constantly commanded to zero
        // Send zero effort command via ros control interce
        actuated_joints_[joint_names_[i]].joint_handle.setCommand(0);
      }
    }
  }

  void MyTiagoController::starting(const ros::Time &time)
  {
    ROS_GREEN_STREAM(" ***** "
                     << "The GCCT has started !"
                     << " ***** ");
  }

  void MyTiagoController::stopping(const ros::Time &time)
  {
    // Set desired effort to zero for the actuated joints
    for (auto it = actuated_joints_.begin(); it != actuated_joints_.end(); it++)
    {
      it->second.joint_handle.setCommand(0);
    }
  }
} // force_control