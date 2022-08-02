#include <XmlRpc.h>

#include "controller.hpp"
#include "ros_color_stream.h"

#include <rbdl/addons/urdfreader/urdfreader.h> ///__RBDL__///

#define URDF_PATH "/opt/pal/ferrum/share/tiago_description/robots/tiagoSteel.urdf"
#define URDF_PATH_2 "/home/pal/tiago_ws/src/tiago_arm_effort_controller/urdf/robot_description.urdf" // /!\ ne peux pas marcher sur le tiago !!!

template <typename T>
bool is_in_vector(const std::vector<T> &vector, const T &elt) {
    return vector.end() != std::find(vector.begin(), vector.end(), elt);
}

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

    ROS_O(">>> Loading MyTiagoController");

    ///__RBDL__///

    // Check int the param server if subchains specified
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

    ROS_R("RBDL joint_names_ after initialization:");
    for (auto i : joint_names_)
        ROS_B(i);

    ///__PINOCCHIO__///

    // Load the urdf model
    std::string path = ros::package::getPath("tiago_arm_effort_controller");
    path = path + "/urdf/robot_description.urdf";
    ROS_R(path);
    const std::string urdf_filename = URDF_PATH;

    ROS_Y(">>> creating the pinocchio model of the whole robot");
    pinocchio::urdf::buildModel(urdf_filename, pin_model_);
    pinocchio::Data pin_data_(pin_model_);

    ROS_Y(">>> parsing the joints to keep in the reduced model");

    // List of joints to keep unlocked by name
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

    ROS_Y(">>> creating the reduced model");
    // Build the reduced model from the list of lock joints
    reduced_model_ = pinocchio::buildReducedModel(pin_model_, list_of_joints_to_lock_by_id, q_rand);
    // pinocchio::buildReducedModel(pin_model_, list_of_joints_to_lock_by_id, q_rand, reduced_model_);
    pinocchio::Data reduced_data_(reduced_model_);

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

    // for (size_t i = 1; i < reduced_model_.njoints; i++)
    //   joint_names_.push_back(reduced_model_.names[i]);

    ///__SHARED__///

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

    // Iinitializa q_act_, q_zero_, tau_cmd_
    q_act_.resize(joint_names_.size());
    q_zero_.resize(joint_names_.size());
    tau_cmd_.resize(joint_names_.size());

    q_act_.setZero();
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

Eigen::VectorXd MyTiagoController::gravityCompensation() {
    // Eigen::VectorXd q = pinocchio::neutral(reduced_model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(reduced_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(reduced_model_.nv);

    pinocchio::Data data(reduced_model_);
    //pinocchio::forwardKinematics(reduced_model_, data, q_act_); // Update the joint placements according to the current joint configuration
    pinocchio::computeAllTerms(reduced_model_,data,q_act_,v);

    const Eigen::VectorXd &tau = pinocchio::rnea(reduced_model_, data, q_act_, v, a);
    // ROS_Y("tau gravity: " << tau.transpose());
    return tau;
}

// Eigen::VectorXd MyTiagoController::poseControl(Eigen::VectorXd dX, Eigen::VectorXd dXdot, Eigen::VectorXd dXddot,
//                                                Eigen::VectorXd X, Eigen::VectorXd Xdot) {
//     const double Kp = 1;
//     const double Kd = 2 * sqrt(Kp);

//     // updating the model
//     pinocchio::Data data(reduced_model_);
//     pinocchio::forwardKinematics(reduced_model_, data, q_act_);

//     // computing the Jacobian of the eof (/!\ change the 7 by a ref)
//     pinocchio::Data::Matrix6x J;
//     pinocchio::computeJointJacobians(reduced_model_, data);
//     pinocchio::getJointJacobian(reduced_model_, data, 7, pinocchio::LOCAL_WORLD_ALIGNED, J);
//     ROS_R("computed Jacombian:" << J);

//     auto tau = ((dX - X) * Kp + (dXdot - Xdot) * Kd + dXddot) * pinocchio::crba(reduced_model_, data, q_act_) * J.inverse();
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
//   auto tau = Jpqp*S*pinocchio::crba(reduced_model_,data, q_act_)*J.inverse()
//               + (fd + (fd-f)*(Kf+KfI)-Xp*Kfd)*(I-S)*J.transpose();
//   return tau;
// }

void MyTiagoController::update(const ros::Time &time, const ros::Duration &period) {
    // Creating the message to be published
    tiago_arm_effort_controller::TorqueCmd msg;
    // TODO: init the message

    // Selecting the controller to use
    static std::string controller = "RBDL"; // "RBDL" or "Pinocchio"/"Pin"

    // Read the current position of all the joints (and velocity)
    // Eigen::VectorXd qdot_act;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_types_[joint_names_[i]] == JointType::ACTUATED ||
            joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL)
            q_act_[i] = actuated_joints_[joint_names_[i]].joint_handle.getPosition();
        else
            q_act_[i] = static_joints_[joint_names_[i]].getPosition();
        // qdot_act[i] = static_joints_[joint_names_[i]].getVelocity();
    }

    ///__RBDL__///

    tau_cmd_.setZero();
    RigidBodyDynamics::InverseDynamics(rbdl_model_, q_act_, q_zero_, q_zero_, tau_cmd_);

    ///__PINOCCHIO__///

    pinocchio::Data reduced_data(reduced_model_);
    pinocchio::forwardKinematics(reduced_model_, reduced_data, q_act_); // Update the joint placements according to the current joint configuration
    auto tau_gravity = gravityCompensation();                           // Compute the minimum torque to maintain the robot at the current position

    // Computes the pose of the end-effector
    pinocchio::Model::Index idx = reduced_model_.existJointName("arm_7_joint") ? reduced_model_.getJointId("arm_7_joint") : (pinocchio::Model::Index)(reduced_model_.njoints - 1);
    auto eof_pose = reduced_data.oMi[idx];
    tiago_arm_effort_controller::EofPose eof_msg;
    eof_msg.x = eof_pose.translation()[0];
    eof_msg.y = eof_pose.translation()[1];
    eof_msg.z = eof_pose.translation()[2];
    auto R = eof_pose.rotation();
    auto rpy = pinocchio::rpy::matrixToRpy(R);
    eof_msg.alpha = rpy(0);
    eof_msg.beta = rpy(1);
    eof_msg.gamma = rpy(2);

    eof_pose_pub_.publish(eof_msg);

    //ROS_R("q_act:" << q_act_);
    pinocchio::forwardKinematics(reduced_model_, reduced_data, q_act_);
    pinocchio::Data::Matrix6x J(6,reduced_model_.nv);
    J.fill(0.);
    pinocchio::computeJointJacobians(reduced_model_, reduced_data);
    pinocchio::getJointJacobian(reduced_model_, reduced_data, reduced_model_.getJointId("arm_7_joint"), pinocchio::LOCAL_WORLD_ALIGNED, J);
    ROS_R("q:\n" << q_act_.transpose());
    ROS_R("J:\n" << J);

    const double Kp = 1;
    const double Kd = 2 * sqrt(Kp);
    Eigen::VectorXd X(6);
    X.fill(0.);

    // desired translations of the eof
    double dXx = 0.1500;// 0.17; // home: 0.1500
    double dXy = 0.1567;// 0.20; // home: 0.1567
    double dXz = 0.5722;// 0.65; // home: 0.5722

    X(0) = dXx - eof_msg.x;
    X(1) = dXy - eof_msg.y;
    X(2) = dXz - eof_msg.z;
    X(3) = 0;
    X(4) = 0;
    X(5) = 0;

    auto X_Kp = (X) * Kp;
    //ROS_B("X_Kp:\n" << X_Kp);

    auto A = pinocchio::crba(reduced_model_, reduced_data, q_act_);
    //ROS_B("A:\n" << A);
    auto Areduced = A.block(1,1,6,6);
    ROS_C("Areduced:\n" << Areduced);


    auto Jreduced = J.block(0,1,6,6);
    ROS_C("Jreduced:\n" << Jreduced);
    auto Jinv = Jreduced.inverse();
    //ROS_B("Jinv:\n" << Jinv);

    auto A_Jinv = Areduced*Jinv;
    ROS_B("A_Jinv:\n" << A_Jinv);

    ROS_R("X_Kp.size: " << X_Kp.rows() << "x" << X_Kp.cols());
    ROS_R("A_Jinv.size: " << A_Jinv.rows() << "x" << A_Jinv.cols());

    auto gamma = A_Jinv*X_Kp;
    ROS_B("tau (Gamma = X_Kp*A_Jinv):\n" << gamma);
    Eigen::VectorXd gamma8(8);
    gamma8.fill(0.);
    for (int i=0; i<6; i++)
        gamma8(i+1)=gamma(i);

    // auto tau_pose = poseControl(q_act_,qdot_act,Eigen::VectorXd::Zero(reduced_model_.nv),q_act_,qdot_act);
    // ROS_R("tau_pose: " << tau_pose);

    auto tau = tau_gravity; // for future torque implementations

    ///__SHARED__///

    // For all the joints...
    for (size_t i = 0; i < joint_names_.size(); ++i) {

        msg.rbdl_gravity_compensation[i] = tau_cmd_[i];
        msg.pin_gravity_compensation[i] = tau(i);
        msg.pin_pose_control[i] = gamma8(i);

        // ...check those one that are actuated
        if (joint_types_[joint_names_[i]] == JointType::ACTUATED) {
            // Translate the calculated torque to desired effort by integrating the frictions of
            // the motor + the ger ration + motor constants
            ActuatedJoint &actuated_joint = actuated_joints_[joint_names_[i]];
            double desired_torque_rbdl = tau_cmd_[i]; ///__RBDL__///
            double desired_torque_pin = tau(i);       ///__PINOCCHIO__///
            // ROS_C("TORQUE RBDL: " << desired_torque_rbdl << " | TORQUE PIN: " << desired_torque_pin);
            double actual_velocity = actuated_joint.joint_handle.getVelocity();

            ///__RBDL__///

            desired_torque_rbdl += actuated_joint.friction_parameters.viscous_friction * actual_velocity;
            if (actual_velocity > actuated_joint.friction_parameters.velocity_tolerance)
                desired_torque_rbdl += actuated_joint.friction_parameters.static_friction;
            else
                desired_torque_rbdl -= actuated_joint.friction_parameters.static_friction;

            double desired_effort_rbdl =
                desired_torque_rbdl / (actuated_joint.actuator_parameters.motor_torque_constant *
                                       actuated_joint.actuator_parameters.reduction_ratio);

            ///__PINOCCHIO__///

            desired_torque_pin += actuated_joint.friction_parameters.viscous_friction * actual_velocity;
            if (actual_velocity > actuated_joint.friction_parameters.velocity_tolerance)
                desired_torque_pin += actuated_joint.friction_parameters.static_friction;
            else
                desired_torque_pin -= actuated_joint.friction_parameters.static_friction;

            double desired_effort_pin =
                desired_torque_pin / (actuated_joint.actuator_parameters.motor_torque_constant *
                                      actuated_joint.actuator_parameters.reduction_ratio);

            ///__SHARED__///

            double desired_effort;

            if (controller == "RBDL") {
                desired_effort = desired_effort_rbdl;
            } else if (controller == "Pinocchio" || controller == "Pin") {
                desired_effort = desired_effort_pin;
            }

            if (std::isnan(desired_effort)) // If desired effort is not valid
            {
                ROS_ERROR_STREAM("Desired effort is not valid for joint "
                                 << joint_names_[i] << " = " << desired_effort);
                return;
            } else {
                // Command an effort to the joint via ros_cotrol interface
                // ROS_O("COMMAND LOOP");

                // double secs = ros::Time::now().toSec();
                // double Fc = 0.15;
                // double temp = 0.3 * sin(2 * 3.141592653 * Fc * secs);
                // double temp2 = 0.3 * (2 * 3.141592653 * Fc) * cos(2 * 3.141592653 * Fc * secs);
                // double temp3 = 0.3 * (2 * 3.141592653 * Fc) * (2 * 3.141592653 * Fc) * -sin(2 * 3.141592653 * Fc * secs);
                // Eigen::VectorXd Xd(6);
                // Xd << temp,0.,0.,0.,0.,0.;
                // Eigen::VectorXd dXd(6);
                // dXd << temp2,0.,0.,0.,0.,0.;
                // Eigen::VectorXd ddXd(6);
                // ddXd << temp3,0.,0.,0.,0.,0.;
                // pinocchio::Data::Matrix6x J(6,pin_model_.nv);
                // J.setZero();
                // pinocchio::Data data(reduced_model_);
                // pinocchio::computeJointJacobian(pin_model_,data,q_act_,7,J);
                // const auto &A = pinocchio::crba(pin_model_,data,q_act_);
                // const Eigen::VectorXd &H = pinocchio::rnea(reduced_model_, data, q_act_, q_zero_, q_zero_);

                // computedTorqueController(Xd, Xd, dXd, dXd, ddXd, ddXd, J, A, H);
                // ROS_Y("TAU[" << i << "]: " << tau.transpose());

                // if (i == 1) {
                //     double secs = ros::Time::now().toSec();
                //     double Fc = 0.15;
                //     desired_effort = 0.3 * sin(2 * 3.141592653 * Fc * secs);
                // }

                /* /!\ effort limiter /!\ */
                auto effortLimit = reduced_model_.effortLimit[i] / 2;
                if (desired_effort > effortLimit) {
                    ROS_R("/!\\ effort " << desired_effort << " limited to " << effortLimit << " /!\\");
                    desired_effort = effortLimit;
                } else if (desired_effort < -effortLimit) {
                    ROS_R("/!\\ effort " << desired_effort << " limited to " << -effortLimit << " /!\\");
                    desired_effort = -effortLimit;
                }

                actuated_joint.joint_handle.setCommand(desired_effort);
            }
        } else if (joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL) {
            // ...and those one that are constantly commanded to zero
            // Send zero effort command via ros control interce
            actuated_joints_[joint_names_[i]].joint_handle.setCommand(0);
        }
    }
    torque_cmd_pub_.publish(msg);
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