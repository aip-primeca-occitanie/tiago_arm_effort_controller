#include <XmlRpc.h>

#include "controller.hpp"
#include "ros_color_stream.h"

#include <rbdl/addons/urdfreader/urdfreader.h>

#define URDF_PATH "/opt/pal/ferrum/share/tiago_description/robots/tiagoSteel.urdf"
#define URDF_PATH_2 "/home/pal/tiago_ws/src/tiago_arm_effort_controller/urdf/robot_description.urdf" // /!\ ne peux pas marcher sur le tiago !!!

template <typename T>
bool is_in_vector(const std::vector<T> &vector, const T &elt) {
    return vector.end() != std::find(vector.begin(), vector.end(), elt);
}

// method for calculating the pseudo-Inverse as recommended by Eigen developers
// template<typename _Matrix_Type_>
// _Matrix_Type_ myPseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
// {
// 	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
// 	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
// 	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
// }

namespace force_control {
bool MyTiagoController::initRequest(hardware_interface::RobotHW *robot_hw,
                                    ros::NodeHandle &root_nh,
                                    ros::NodeHandle &controller_nh,
                                    ClaimedResources &claimed_resources) {
    ROS_O(">>> Initializing MyTiagoController");
    // Check if construction finished cleanly
    if (state_ != CONSTRUCTED) {
        ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
        return false;
    }

    // Get a pointer to the joint effort control interface
    hardware_interface::EffortJointInterface *effort_iface =
        robot_hw->get<hardware_interface::EffortJointInterface>();

    if (!effort_iface) {
        ROS_ERROR("This controller requires a hardware interface of type EffortJointInterface."
                  " Make sure this is registered in the hardware_interface::RobotHW class.");
        return false;
    }

    // Get a pointer to the joint position control interface
    hardware_interface::JointStateInterface *joint_state_iface =
        robot_hw->get<hardware_interface::JointStateInterface>();
    if (!joint_state_iface) {
        ROS_ERROR("This controller requires a hardware interface of type JointStateInterface."
                  " Make sure this is registered in the hardware_interface::RobotHW class.");
        return false;
    }

    // Clear resources associated at both interfaces
    effort_iface->clearClaims();
    joint_state_iface->clearClaims();

    if (!init(effort_iface, joint_state_iface, root_nh, controller_nh)) {
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
                             ros::NodeHandle & /*root_nh*/, ros::NodeHandle &control_nh) {

    torque_cmd_pub_ = control_nh.advertise<tiago_arm_effort_controller::TorqueCmd>("torque_cmd", 1);
    eof_pose_pub_ = control_nh.advertise<tiago_arm_effort_controller::EofPose>("eof_pose", 1);
    print_pub_ = control_nh.advertise<tiago_arm_effort_controller::Print>("print", 1);
    teleop_sub_ = control_nh.subscribe("teleop", 10, &MyTiagoController::teleopCallBack, this);

    /*Initialize the controller options*/
    controller_lib_ = RBDL;
    controller_type_ = GC;

    Kp_ = 1;

    ROS_O(">>> Loading MyTiagoController");

    /*__RBDL__*/

    // Check in the param server if subchains specified
    std::vector<std::string> tip_links;
    control_nh.getParam("robot_model_chains", tip_links);

    std::vector<double> joint_position_min;
    std::vector<double> joint_position_max;
    std::vector<double> joint_vel_min;
    std::vector<double> joint_vel_max;
    std::vector<double> joint_damping;
    std::vector<double> joint_friction;
    std::vector<double> joint_max_effort;

    if (tip_links.size() > 0) {
        // Parse the robot if subchains specified
        RigidBodyDynamics::Addons::URDFReadFromParamServer(
            &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, tip_links,
            joint_names_, joint_position_min, joint_position_max, joint_vel_min,
            joint_vel_max, joint_damping, joint_friction, joint_max_effort);
    } else {
        // Parse the full robot if there is no subchain specified
        RigidBodyDynamics::Addons::URDFReadFromParamServer(
            &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, joint_names_,
            joint_position_min, joint_position_max, joint_vel_min, joint_vel_max,
            joint_damping, joint_friction, joint_max_effort);
    }

    ROS_M("RBDL joint_names_ after initialization:");
    for (auto i : joint_names_)
        ROS_B(i);

    /*__PINOCCHIO__*/

    /*Building Pinocchio model*/
    // Load the urdf model
    std::string path = ros::package::getPath("tiago_arm_effort_controller");
    path = path + "/urdf/robot_description.urdf";
    ROS_R("urdf path: " << path);
    const std::string urdf_filename = URDF_PATH;
    // Create the pinocchio model
    ROS_Y(">>> creating the pinocchio model of the whole robot");
    pinocchio::urdf::buildModel(urdf_filename, pin_model_);
    pinocchio::Data pin_data_(pin_model_);

    /*Building Pinocchio reduced model*/
    // List of joints to keep unlocked by name
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
    // List of joints to keep unlocked by ID
    std::vector<pinocchio::JointIndex> list_of_joints_to_keep_unlocked_by_id;
    Eigen::VectorXd q_rand = pinocchio::randomConfiguration(pin_model_);
    for (std::vector<std::string>::const_iterator it = list_of_joints_to_keep_unlocked_by_name.begin();
         it != list_of_joints_to_keep_unlocked_by_name.end(); ++it) {
        const std::string &joint_name = *it;
        if (pin_model_.existJointName(joint_name))
            list_of_joints_to_keep_unlocked_by_id.push_back(pin_model_.getJointId(joint_name));
        else
            std::cout << "joint: " << joint_name << " does not belong to the model";
    }
    // Transform the list into a list of joints to lock
    std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
    for (pinocchio::JointIndex joint_id = 1; joint_id < pin_model_.joints.size(); ++joint_id) {
        const std::string joint_name = pin_model_.names[joint_id];
        if (is_in_vector(list_of_joints_to_keep_unlocked_by_name, joint_name))
            continue;
        else {
            list_of_joints_to_lock_by_id.push_back(joint_id);
        }
    }
    // Build the reduced model from the list of lock joints
    ROS_Y(">>> creating the reduced model");
    reduced_model_ = pinocchio::buildReducedModel(pin_model_, list_of_joints_to_lock_by_id, q_rand);

    // Display the parameters of the reduced model for the TIAGo's arm
    ROS_M("Reduced Model Parameters:");
    ROS_B("joints (name: id): " << reduced_model_.njoints);
    for (size_t i = 0; i < reduced_model_.njoints; i++)
        ROS_C(reduced_model_.names[i] << ": " << reduced_model_.joints[i].id());
    ROS_B("lowerPositionLimit: " << reduced_model_.lowerPositionLimit.size());
    for (size_t i = 0; i < reduced_model_.lowerPositionLimit.size(); i++)
        ROS_C(reduced_model_.lowerPositionLimit[i]);
    ROS_B("upperPositionLimit: " << reduced_model_.upperPositionLimit.size());
    for (size_t i = 0; i < reduced_model_.upperPositionLimit.size(); i++)
        ROS_C(reduced_model_.upperPositionLimit[i]);
    ROS_B("velocityLimit: " << reduced_model_.velocityLimit.size());
    for (size_t i = 0; i < reduced_model_.velocityLimit.size(); i++)
        ROS_C(reduced_model_.velocityLimit[i]);
    ROS_B("damping: " << reduced_model_.damping.size());
    for (size_t i = 0; i < reduced_model_.damping.size(); i++)
        ROS_C(reduced_model_.damping[i]);
    ROS_B("friction: " << reduced_model_.friction.size());
    for (size_t i = 0; i < reduced_model_.friction.size(); i++)
        ROS_C(reduced_model_.friction[i]);
    ROS_B("effortLimit: " << reduced_model_.effortLimit.size());
    for (size_t i = 0; i < reduced_model_.effortLimit.size(); i++)
        ROS_C(reduced_model_.effortLimit[i]);

    /*__COMMON__*/

    for (size_t i = 0; i < joint_names_.size(); i++) {
        // Checks joint type from param server
        std::string control_type;
        if (!control_nh.getParam("joints/" + joint_names_[i] + "/type", control_type)) {
            ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " interface type");
            return false;
        }

        if (control_type == "actuated" ||
            control_type == "no_control") // If joint is actuated or constantly commanded to zero
        {
            // Read the actuator parameters from param server
            ActuatorParameters actuator_parameters;
            if (!control_nh.getParam("joints/" + joint_names_[i] + "/motor_torque_constant",
                                     actuator_parameters.motor_torque_constant)) {
                ROS_ERROR_STREAM("Could not find motor torque constant for joint " << joint_names_[i]);
                return false;
            }
            if (!control_nh.getParam("joints/" + joint_names_[i] + "/reduction_ratio",
                                     actuator_parameters.reduction_ratio)) {
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

            try {
                // Try to get an effort interface handle to command the joint in effort
                hardware_interface::JointHandle joint_handle =
                    effort_iface->getHandle(joint_names_[i]);
                // Creates an actuated joint and insert in the map of actuated joints
                ActuatedJoint actuated_joint;
                actuated_joint.joint_handle = joint_handle;
                actuated_joint.actuator_parameters = actuator_parameters;
                actuated_joint.friction_parameters = friction_parameters;
                actuated_joints_.insert(std::make_pair(joint_names_[i], actuated_joint));
            } catch (...) {
                ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Effort interface");
                return false;
            }
            // Insert the joint in the map of joint types according to his type
            if (control_type == "actuated")
                joint_types_.insert(std::make_pair(joint_names_[i], JointType::ACTUATED));
            else if (control_type == "no_control")
                joint_types_.insert(std::make_pair(joint_names_[i], JointType::ACTUATED_NO_CONTROL));
        } else // If static joint
        {
            try {
                // Try to get a joint state handle which only allows us to read the current states
                // of the joint
                hardware_interface::JointStateHandle joint_state_handle =
                    joint_state_iface->getHandle(joint_names_[i]);
                // Insert this handle in the map of static joints
                static_joints_.insert(std::make_pair(joint_names_[i], joint_state_handle));
            } catch (...) {
                ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Position interface");
                return false;
            }
            // Insert the joint in the map of joint types
            joint_types_.insert(std::make_pair(joint_names_[i], JointType::STATIC));
        }
    }

    assert(joint_types_.size() == joint_names_.size());
    assert(joint_types_.size() == actuated_joints_.size() + static_joints_.size());

    // Iinitializa q_mesured_, q_zero_, tau_cmd_
    q_mesured_.resize(joint_names_.size());
    q_zero_.resize(joint_names_.size());
    tau_cmd_.resize(joint_names_.size());

    q_mesured_.setZero();
    q_zero_.setZero();
    tau_cmd_.setZero();

    // Allows to modify the gravity compensation parameters from the rqt reconfigure
    for (auto it = actuated_joints_.begin(); it != actuated_joints_.end(); it++) {
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

Eigen::VectorXd MyTiagoController::gravityCompensation(pinocchio::Data data) {
    // Create zeros matrix for velocity and acceleration
    static const Eigen::VectorXd v = Eigen::VectorXd::Zero(reduced_model_.nv);
    static const Eigen::VectorXd a = Eigen::VectorXd::Zero(reduced_model_.nv);
    // Compute the torque with the rnea
    const Eigen::VectorXd &tau = pinocchio::rnea(reduced_model_, data, q_mesured_, v, a);
    return tau;
}

Eigen::VectorXd MyTiagoController::poseControl_q(Eigen::VectorXd qd) {
    // Static gains
    static const double Kp = 10;
    static const double Ki = 0.0002;
    // Epsilon q (Integrator)
    static Eigen::VectorXd epsilon_q = Eigen::VectorXd::Zero(8);

    auto delta_q = qd - q_mesured_;
    // ROS_C("delta_q:\n" << delta_q);
    epsilon_q = epsilon_q + delta_q;
    auto tau = Kp * (delta_q) + Ki * (epsilon_q);
    return tau;
}

Eigen::VectorXd MyTiagoController::poseControl_X(Eigen::VectorXd X, Eigen::VectorXd Xd, pinocchio::Data::Matrix6x J) {
    // Static gains
    static const double Kp = 1e1;
    static const double Ki = 1e-5;
    // Epsilon q (Integrator)
    static Eigen::VectorXd epsilon_q = Eigen::VectorXd::Zero(7);

    auto delta_X = Xd - X;
    Eigen::VectorXd delta_X_reduced = Eigen::VectorXd::Zero(3);
    delta_X_reduced(0) = delta_X(0);
    delta_X_reduced(1) = delta_X(1);
    delta_X_reduced(2) = delta_X(2);
    // ROS_G("delta_X_reduced:\n" << delta_X_reduced);

    // ROS_G("J:\n" << J);
    auto J_reduced = J.block(0, 1, 3, 7);

    // ROS_G("J_reduced:\n" << J_reduced);
    Eigen::MatrixXd Jpinv = J_reduced.completeOrthogonalDecomposition().pseudoInverse();
    ROS_C("Jpinv:\n" << Jpinv);

    
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J_reduced);
    // auto Jpinv2 = cqr.pseudoInverse();
    // ROS_C("Jpinv2:\n" << Jpinv2 << Jpinv.rows() << Jpinv.cols());

    auto delta_q = Jpinv * delta_X_reduced;
    ROS_C("delta_q:\n" << delta_q);

    auto delta_q_solve = cqr.solve(delta_X_reduced);
    ROS_G("delta_q_solve:\n" << delta_q_solve);
    epsilon_q = epsilon_q + delta_q_solve;
    // ROS_G("epsilon_q:\n" << epsilon_q);
    auto tau = Kp * (delta_q_solve) + Ki * (epsilon_q);
    Eigen::VectorXd tau_8 = Eigen::VectorXd::Zero(8);
    for (int i = 1; i < 8; i++)
        tau_8(i) = tau(i - 1);
    return tau_8;
}

Eigen::VectorXd MyTiagoController::poseControl_X_teleop(Eigen::VectorXd X, Eigen::VectorXd Xd, pinocchio::Data::Matrix6x J) {
    // Static gains
    // /!\ Using Kp_ !!!
    static const double Ki = 1e-5;
    // Epsilon q (Integrator)
    static Eigen::VectorXd epsilon_q = Eigen::VectorXd::Zero(7);

    auto delta_X = Xd - X;
    Eigen::VectorXd delta_X_reduced = Eigen::VectorXd::Zero(3);
    delta_X_reduced(0) = delta_X(0);
    delta_X_reduced(1) = delta_X(1);
    delta_X_reduced(2) = delta_X(2);

    auto J_reduced = J.block(0, 1, 3, 7);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J_reduced);
    auto delta_q_solve = cqr.solve(delta_X_reduced);
    epsilon_q = epsilon_q + delta_q_solve;
    auto tau = Kp_ * (delta_q_solve) + Ki * (epsilon_q);
    Eigen::VectorXd tau_8 = Eigen::VectorXd::Zero(8);
    for (int i = 1; i < 8; i++)
        tau_8(i) = tau(i - 1);
    return tau_8;
}

// Eigen::VectorXd MyTiagoController::poseControl(Eigen::VectorXd dX, Eigen::VectorXd dXdot, Eigen::VectorXd dXddot,
//                                                Eigen::VectorXd delta_X, Eigen::VectorXd Xdot) {
//     const double Kp = 1;
//     const double Kd = 2 * sqrt(Kp);

//     // updating the model
//     pinocchio::Data data(reduced_model_);
//     pinocchio::forwardKinematics(reduced_model_, data, q_mesured_);

//     // computing the Jacobian of the eof (/!\ change the 7 by a ref)
//     pinocchio::Data::Matrix6x J;
//     pinocchio::computeJointJacobians(reduced_model_, data);
//     pinocchio::getJointJacobian(reduced_model_, data, 7, pinocchio::LOCAL_WORLD_ALIGNED, J);
//     ROS_R("computed Jacombian:" << J);

//     auto tau = ((dX - delta_X) * Kp + (dXdot - Xdot) * Kd + dXddot) * pinocchio::crba(reduced_model_, data, q_mesured_) * J.inverse();
//     return tau;
// }

// Eigen::VectorXd MyTiagoController::forceControl(Eigen::VectorXd fd, Eigen::MatrixXd J, pinocchio::Data data)
// {
//   const double Kf = 1;
//   const double KfI = 1;
//   const double Kfd = 1;
//   Eigen::VectorXd Xp;
//   Eigen::VectorXd qp;
//   Eigen::VectorXd f;
//   Eigen::MatrixXd S;

//   Eigen::VectorXd Jpqp = J.derived()*qp;
//   auto I = S.Identity(); // /!\ does that work ???
//   auto tau = Jpqp*S*pinocchio::crba(reduced_model_,data, q_mesured_)*J.inverse()
//               + (fd + (fd-f)*(Kf+KfI)-Xp*Kfd)*(I-S)*J.transpose();
//   return tau;
// }

void MyTiagoController::teleopCallBack(const tiago_arm_effort_controller::Teleop &msg) {
    controller_lib_ = (DynLib)msg.control_lib;
    controller_type_ = (ControlType)msg.control_type;

    ROS_Y("Teleop command:\nDynamic Library: " << dyn_lib_[msg.control_lib]
                                               << "\nController Type: "
                                               << control_type_[msg.control_type]
                                               << "\nGain: " << msg.coeff_cmd);
}

void MyTiagoController::update(const ros::Time &time, const ros::Duration &period) {

    /*Read the current position of all the joints*/
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_types_[joint_names_[i]] == JointType::ACTUATED ||
            joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL)
            q_mesured_[i] = actuated_joints_[joint_names_[i]].joint_handle.getPosition();
        else
            q_mesured_[i] = static_joints_[joint_names_[i]].getPosition();
    }

    /*__RBDL__*/

    // Use rbdl's rnea to compute the base torque compensating gravuty
    tau_cmd_.setZero();
    RigidBodyDynamics::InverseDynamics(rbdl_model_, q_mesured_, q_zero_, q_zero_, tau_cmd_);

    /*__PINOCCHIO__*/

    // Create the data to be used at each update
    pinocchio::Data reduced_data(reduced_model_);
    // Update the joint placements according to the current joint configuration
    pinocchio::forwardKinematics(reduced_model_, reduced_data, q_mesured_);
    // Compute the minimum torque to maintain the robot at the current position
    auto tau_GC = gravityCompensation(reduced_data);

    // Computes the pose of the end-effector
    pinocchio::Model::Index idx_eof = reduced_model_.getJointId("arm_7_joint");
    auto X_data = reduced_data.oMi[idx_eof];
    Eigen::VectorXd X = Eigen::VectorXd::Zero(6);
    X(0) = X_data.translation()[0];
    X(1) = X_data.translation()[1];
    X(2) = X_data.translation()[2];
    auto X_rpy = pinocchio::rpy::matrixToRpy(X_data.rotation());
    X(3) = X_rpy(0);
    X(4) = X_rpy(1);
    X(5) = X_rpy(2);

    // Message of the pose of the end effector to be published on "/eof_pose"
    tiago_arm_effort_controller::EofPose eof_msg;
    eof_msg.x = X(0);
    eof_msg.y = X(1);
    eof_msg.z = X(2);
    eof_msg.alpha = X(3);
    eof_msg.beta = X(4);
    eof_msg.gamma = X(5);
    eof_pose_pub_.publish(eof_msg);

    // Compute the Jacobian of the eof in the LOCAL_WORLD_ALIGNED frame
    pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, reduced_model_.nv);
    pinocchio::computeJointJacobians(reduced_model_, reduced_data);
    pinocchio::getJointJacobian(reduced_model_, reduced_data, idx_eof, pinocchio::LOCAL_WORLD_ALIGNED, J);
    auto point_position = Eigen::Vector3d::Zero();
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(3, reduced_model_.nv);
    RigidBodyDynamics::CalcPointJacobian(rbdl_model_, q_mesured_, 7, point_position, G);
    ROS_R("J:\n"
          << J);
    ROS_R("G:\n"
          << G);

    /*Pose control through a desired q*/
    // Set desired q to [0,0,0,0,0,0,0,0] (arm stretched to the right)
    // static const auto qd = q_mesured_;
    static const Eigen::VectorXd qd = (Eigen::VectorXd(8) << 0, 0, 0, 0, 1.57, 0, 0, 0).finished();
    auto tau_PC_q = poseControl_q(qd);

    /*Pose control through a desired X*/
    // Set desired X to [.114, -.791, .695, #, #, #] (arm stretched to the right)
    // Home position is [.1500, .1567, 0.5722, #, #, #]
    static Eigen::VectorXd Xd = (Eigen::VectorXd(6) << .114, -.791, .695, 0, 0, 0).finished();
    static const Eigen::VectorXd Xinit = X; // To be used as Xd
    auto tau_PC_X = poseControl_X(X, Xd, J);
    // ROS_Y("X:\n" << X);
    // ROS_Y("Xd:\n" << Xd);
    // ROS_R("tau_PC_X:\n" << tau_PC_X);
    // ROS_O("tau_GC:\n" << tau_cmd_);

    Eigen::MatrixXd A = (Eigen::MatrixXd(3,4) << 5, 2, 3, 3, 4, 5, 6, 1, 9, 1, 8, 4).finished();
    Eigen::MatrixXd Apinv = A.completeOrthogonalDecomposition().pseudoInverse();
    ROS_R("Apinv:\n" << Apinv << Apinv.rows() << Apinv.cols());

    /*__COMMON__*/

    /*Set the final tau to send to the robot*/
    static Eigen::VectorXd tau = Eigen::VectorXd::Zero(joint_names_.size());
    // Choose the dynamic library for the RNEA
    switch (controller_lib_) {
    case Pin:
        tau = tau_GC;
        break;
    case RBDL:
        tau = tau_cmd_;
    default:
        break;
    }
    // Choose the type of controller used
    switch (controller_type_) {
    case GC:
        break;
    case PC_q:
        tau += tau_PC_q;
        break;
    case PC_X:
        tau += tau_PC_X;
        break;
    case DC:
        ROS_R(">>> Not yet implemented, defaulting to GC");
        break;
    default:
        ROS_R(">>> Unknown controller type, defaulting to GC");
        break;
    }

    // tau = tau_cmd_; /* /!\ force tau for debug !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    /*Sending the print_msg to the topic "/print"*/
    tiago_arm_effort_controller::Print print_msg;
    for (int i = 0; i < 8; i++)
        print_msg.qd[i] = qd(i);
    for (int i = 0; i < 8; i++)
        print_msg.q[i] = q_mesured_(i);
    for (int i = 0; i < 6; i++)
        print_msg.Xd[i] = Xd(i);
    for (int i = 0; i < 6; i++)
        print_msg.X[i] = X(i);
    print_pub_.publish(print_msg);

    /*Creating the message of the torques to be published*/
    tiago_arm_effort_controller::TorqueCmd torque_cmd_msg;

    // For all the joints...
    for (size_t i = 0; i < joint_names_.size(); ++i) {

        // Create the torque message to be sent
        torque_cmd_msg.rbdl_gravity_compensation[i] = tau_cmd_(i);
        torque_cmd_msg.pin_gravity_compensation[i] = tau_GC(i);

        // ...check those one that are actuated
        if (joint_types_[joint_names_[i]] == JointType::ACTUATED) {
            // Translate the calculated torque to desired effort by integrating the frictions of
            // the motor + the ger ration + motor constants
            ActuatedJoint &actuated_joint = actuated_joints_[joint_names_[i]];
            double actual_velocity = actuated_joint.joint_handle.getVelocity();

            /* Compute the desired effort*/
            double desired_torque = tau[i];

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
            } else {
                /*Effort limiter*/
                auto effortLimit = reduced_model_.effortLimit[i] * 0.9;
                if (desired_effort > effortLimit) {
                    ROS_R("/!\\ effort " << desired_effort << " limited to " << effortLimit << " /!\\");
                    ROS_B(i);
                    desired_effort = effortLimit;
                } else if (desired_effort < -effortLimit) {
                    ROS_R("/!\\ effort " << desired_effort << " limited to " << -effortLimit << " /!\\");
                    ROS_B(i);
                    desired_effort = -effortLimit;
                }

                /*Sending the desired effort command to the robot*/
                actuated_joint.joint_handle.setCommand(desired_effort);
            }
        } else if (joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL) {
            // ...and those one that are constantly commanded to zero
            // Send zero effort command via ros control interce
            actuated_joints_[joint_names_[i]].joint_handle.setCommand(0);
        }
    }
    /*Sending the torque command message to the topic "/torque_cmd"*/
    torque_cmd_pub_.publish(torque_cmd_msg);
}

void MyTiagoController::starting(const ros::Time &time) {
    ROS_O(">>> Start of the tiago_arm_effort_controller");
}

void MyTiagoController::stopping(const ros::Time &time) {
    // Set desired effort to zero for the actuated joints
    for (auto it = actuated_joints_.begin(); it != actuated_joints_.end(); it++) {
        it->second.joint_handle.setCommand(0);
    }
}
} // namespace force_control