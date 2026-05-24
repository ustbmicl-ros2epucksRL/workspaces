#include "traj_opti/traj_opti.h"

#include <algorithm>
#include <cmath>
#include <exception>

namespace {

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double clampValue(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

}

/*==========================================*/
/*             TrajectoryOptimizer          */
/*==========================================*/

TrajectoryOptimizer::TrajectoryOptimizer() {
    state = Eigen::Vector3d::Zero();
    goal = Eigen::Vector2d::Zero();
    global_goal = Eigen::Vector2d::Zero();
    vel = {0.0, 0.0};
}

TrajectoryOptimizer::TrajectoryOptimizer(ros::NodeHandle &nh_) {
    nh_.param("optimization/goal_distance_thresh", dist_thresh, -1.0);
    nh_.param("controller/stall_fallback_enabled", stall_fallback_enabled, false);
    nh_.param("controller/stall_fallback_goal_distance", stall_fallback_goal_distance, 0.35);
    nh_.param("controller/stall_fallback_linear_epsilon", stall_fallback_linear_epsilon, 0.015);
    nh_.param("controller/fallback_other_clearance", fallback_other_clearance, 0.4);
    nh_.param("controller/fallback_repulsion_gain", fallback_repulsion_gain, 1.0);
    state = Eigen::Vector3d::Zero();
    goal = Eigen::Vector2d::Zero();
    global_goal = Eigen::Vector2d::Zero();
    vel = {0.0, 0.0};
    nmpc_.reset(new NMPCController(nh_));

//    nmpc_->initialize();
    nmpc_->initializeInHighCollision();

    cur_cost_type = NORMAL_MODE;

}

void TrajectoryOptimizer::setLocalGoal(Eigen::Vector2d local_goal, Eigen::Vector2d desired_wv) {
    safeSetLocalGoal(local_goal, desired_wv);
}

bool TrajectoryOptimizer::safeSetLocalGoal(Eigen::Vector2d local_goal, Eigen::Vector2d desired_wv) {
    goal = local_goal;
    x_ref = {local_goal.x(), local_goal.y(), 0.0};
    u_ref = {desired_wv.x(), desired_wv.y()};

    if (!isFiniteVector(local_goal) || !isFiniteVector(desired_wv)) {
        ROS_WARN_THROTTLE(1.0,
                          "Reject invalid local goal/reference: goal=(%.3f, %.3f), u=(%.3f, %.3f)",
                          local_goal.x(),
                          local_goal.y(),
                          desired_wv.x(),
                          desired_wv.y());
        vel = fallbackVelocity();
        return false;
    }

    try {
        nmpc_->setReference(x_ref, u_ref);
    } catch (casadi::CasadiException& e) {
        ROS_WARN_THROTTLE(1.0, "NMPC rejected reference, using fallback controller: %s", e.what());
        vel = fallbackVelocity();
        return false;
    } catch (std::exception& e) {
        ROS_WARN_THROTTLE(1.0, "NMPC reference update failed, using fallback controller: %s", e.what());
        vel = fallbackVelocity();
        return false;
    }

    return true;
}

void TrajectoryOptimizer::setGlobalGoal(Eigen::Vector2d glb_goal) {
    if (!isFiniteVector(glb_goal)) {
        ROS_WARN_THROTTLE(1.0,
                          "Reject invalid global goal: (%.3f, %.3f)",
                          glb_goal.x(),
                          glb_goal.y());
        return;
    }
    global_goal = glb_goal;
}

void TrajectoryOptimizer::updateState(Eigen::Vector3d s, Eigen::Vector2d c) {
   if (!isFiniteVector(s) || !isFiniteVector(c)) {
       ROS_WARN_THROTTLE(1.0,
                         "Reject invalid NMPC state/control init: state=(%.3f, %.3f, %.3f), u=(%.3f, %.3f)",
                         s.x(),
                         s.y(),
                         s.z(),
                         c.x(),
                         c.y());
       return;
   }

   state = s;
   std::vector<double> current_control = {c.x(), c.y()};
   std::vector<double> current_state = {s.x(), s.y(), s.z()};
   try {
       nmpc_->setXUInit(current_state, current_control);
   } catch (casadi::CasadiException& e) {
       ROS_WARN_THROTTLE(1.0, "NMPC state update failed, keeping previous state init: %s", e.what());
   } catch (std::exception& e) {
       ROS_WARN_THROTTLE(1.0, "NMPC state update failed, keeping previous state init: %s", e.what());
   }
}

void TrajectoryOptimizer::updateOtherOdom(Eigen::Vector2d pos, int id) {
    if (!isFiniteVector(pos)) {
        return;
    }
    if (id >= 0) {
        if (static_cast<std::size_t>(id) >= other_positions_.size()) {
            other_positions_.resize(static_cast<std::size_t>(id) + 1, Eigen::Vector2d(500.0, 500.0));
        }
        other_positions_[id] = pos;
    }
    try {
        nmpc_->setOthersState(pos,id);
    } catch (casadi::CasadiException& e) {
        ROS_WARN_THROTTLE(1.0, "NMPC other-robot update failed: %s", e.what());
    } catch (std::exception& e) {
        ROS_WARN_THROTTLE(1.0, "NMPC other-robot update failed: %s", e.what());
    }
}

void TrajectoryOptimizer::toggleCostFunc(int costFuncType) {

    if (costFuncType == cur_cost_type)
        return;

    if (costFuncType == HIGH_COLLISION_MODE)
        nmpc_->initializeInHighCollision();
    else if (costFuncType == NORMAL_MODE)
        nmpc_->initialize();
    else
        ROS_WARN("valid costFuncType Value");
}

bool TrajectoryOptimizer::run() {

    vel.clear();
    vel.resize(2);

    Eigen::Vector2d tb_pos(state.x(), state.y());
    Eigen::Vector2d goal_pos(global_goal.x(), global_goal.y());
    if (!isFiniteVector(state) || !isFiniteVector(goal_pos)) {
        vel = fallbackVelocity();
        return false;
    }
    double dist = (goal_pos - tb_pos).norm();
    if (dist < dist_thresh) {
        //finish goal
        vel = {0.0, 0.0};
        return true;
    }
    else {
        try {
            nmpc_->solve();
            vel = nmpc_->getu();
            if (!isFiniteVector(vel, 2)) {
                ROS_WARN_THROTTLE(1.0, "NMPC returned invalid velocity, using fallback controller.");
                vel = fallbackVelocity();
            } else if (stall_fallback_enabled) {
                const Eigen::Vector2d local_delta = goal - tb_pos;
                const double local_goal_dist = local_delta.norm();
                if (std::isfinite(local_goal_dist) &&
                    local_goal_dist > stall_fallback_goal_distance &&
                    std::abs(vel[0]) < stall_fallback_linear_epsilon) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "NMPC returned near-zero linear velocity %.3f while local goal is %.2fm away; using fallback controller.",
                        vel[0],
                        local_goal_dist);
                    vel = fallbackVelocity();
                }
            }
        }
        catch (casadi::CasadiException& e) {
            vel = fallbackVelocity();
            std::cout << std::endl;
            std::cout << e.what() << std::endl;
            std::cout << std::endl;
        } catch (std::exception& e) {
            vel = fallbackVelocity();
            ROS_WARN_THROTTLE(1.0, "NMPC solve failed, using fallback controller: %s", e.what());
        }
        return false;
    }
}

std::vector<double> TrajectoryOptimizer::getControlVelocity() {
    if (!isFiniteVector(vel, 2)) {
        return {0.0, 0.0};
    }
    return vel;
}

std::vector<double> TrajectoryOptimizer::getState() {
    return nmpc_->getState();
}

std::vector<double> TrajectoryOptimizer::fallbackVelocity() const {
    if (!std::isfinite(state.x()) || !std::isfinite(state.y()) || !std::isfinite(state.z())) {
        return {0.0, 0.0};
    }

    const Eigen::Vector2d tb_pos(state.x(), state.y());
    Eigen::Vector2d delta = goal - tb_pos;
    if (!std::isfinite(delta.x()) || !std::isfinite(delta.y()) || delta.norm() < 1e-3) {
        delta = global_goal - tb_pos;
    }
    if (!std::isfinite(delta.x()) || !std::isfinite(delta.y()) || delta.norm() < 1e-3) {
        return {0.0, 0.0};
    }

    const double dist = delta.norm();
    Eigen::Vector2d desired_direction = delta / dist;
    Eigen::Vector2d repulsion = Eigen::Vector2d::Zero();
    const double clearance = std::max(0.0, fallback_other_clearance);

    if (clearance > 1e-3) {
        for (const auto& other_position : other_positions_) {
            if (!std::isfinite(other_position.x()) || !std::isfinite(other_position.y())) {
                continue;
            }
            Eigen::Vector2d away = tb_pos - other_position;
            const double other_dist = away.norm();
            if (!std::isfinite(other_dist) || other_dist >= clearance) {
                continue;
            }
            if (other_dist < 1e-3) {
                away = Eigen::Vector2d(-desired_direction.y(), desired_direction.x());
            } else {
                away /= other_dist;
            }
            repulsion += away * ((clearance - std::max(other_dist, 0.0)) / clearance);
        }
    }

    if (repulsion.norm() > 1e-6) {
        const double gain = std::max(0.0, fallback_repulsion_gain);
        Eigen::Vector2d combined = desired_direction + gain * repulsion;
        if (combined.norm() > 1e-6) {
            desired_direction = combined.normalized();
        }
    }

    const double desired_yaw = std::atan2(desired_direction.y(), desired_direction.x());
    const double yaw_error = normalizeAngle(desired_yaw - state.z());

    double linear = clampValue(0.45 * dist, 0.02, 0.12);
    if (repulsion.norm() > 1e-6) {
        linear = std::min(linear, 0.08);
    }
    if (std::abs(yaw_error) > 1.2) {
        linear = 0.0;
    } else if (std::abs(yaw_error) > 0.6) {
        linear *= 0.5;
    }
    const double angular = clampValue(1.2 * yaw_error, -1.2, 1.2);

    return {linear, angular};
}

bool TrajectoryOptimizer::isFiniteVector(const Eigen::Vector2d& value) const {
    return std::isfinite(value.x()) && std::isfinite(value.y());
}

bool TrajectoryOptimizer::isFiniteVector(const Eigen::Vector3d& value) const {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

bool TrajectoryOptimizer::isFiniteVector(const std::vector<double>& value, std::size_t expected_size) const {
    if (value.size() < expected_size) {
        return false;
    }
    for (std::size_t i = 0; i < expected_size; ++i) {
        if (!std::isfinite(value[i])) {
            return false;
        }
    }
    return true;
}

/*==========================================*/
/*             NMPCController          */
/*==========================================*/

NMPCController::NMPCController() : T_(40), nx_(3), nu_(2), dt_(0.1) {

}

NMPCController::NMPCController(ros::NodeHandle &nh_) : nx_(3), nu_(2){
    nh_.param("controller/T", T_, -1);
    nh_.param("controller/dt", dt_, -0.1);
    nh_.param("controller/distance_other_tb_dist", dist_other_th, -0.1);
    nh_.param("controller/other_num_in_opti", other_num, -1);
    other_constraint_horizon = T_;
}

void NMPCController::initialize() {
    x_ = opti_.variable(nx_, T_+1);
    u_ = opti_.variable(nu_, T_);

    _x_ref = opti_.parameter(nx_, T_+1);
    _u_ref = opti_.parameter(nu_, T_);

    p_ = opti_.parameter(nx_, 1);
    Q = opti_.parameter(nx_, nx_);
    R = opti_.parameter(nu_, nu_);

    J_ = casadi::MX::dot(x_ - _x_ref, casadi::MX::mtimes(Q, x_ - _x_ref)) +
         casadi::MX::dot(u_ - _u_ref, casadi::MX::mtimes(R,u_ - _u_ref));

    opti_.minimize(J_);

    // Define the control horizons
    for (int i = 0; i < T_; i++) {
        opti_.subject_to(x_(casadi::Slice(), i + 1) == x_(casadi::Slice(), casadi::Slice(i)) + \
                                                         dynamics_(x_(casadi::Slice(), casadi::Slice(i)), u_(casadi::Slice(), casadi::Slice(i)), dt_));
    }

    // constraint u and it derivative
    for (int i = 0; i < T_ - 1; i++) {
        auto devel = u_(casadi::Slice(), i + 1) - u_(casadi::Slice(), i) / dt_;
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_VELOCITY, devel(0), MAX_DELTA_VELOCITY));
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_OMEGA, devel(1), MAX_DELTA_OMEGA));
    }

    opti_.subject_to(casadi::Opti::bounded(0.0, u_(0), MAX_LINEAR_VELOCITY));
    opti_.subject_to(casadi::Opti::bounded(-MAX_ANGULAR_VELOCITY, u_(1), MAX_ANGULAR_VELOCITY));

    // constraint x
    opti_.subject_to(casadi::Opti::bounded(-M_PI, x_(2), M_PI));
    opti_.subject_to(x_(casadi::Slice(), 0) == p_);

    solver_options_["ipopt.print_level"] = 0;
    solver_options_["ipopt.sb"] = "yes";
    solver_options_["ipopt.max_iter"] = 2000;
    solver_options_["ipopt.tol"] = 1e-8;
    solver_options_["print_time"] = 0;
    solver_options_["ipopt.acceptable_obj_change_tol"] = 1e-6;
    opti_.solver("ipopt", solver_options_);

    // state and control weight
    opti_.set_value(Q, casadi::DM::diag(casadi::DM::vertcat({1.0, 1.0, 0.0})));
    opti_.set_value(R, casadi::DM::diag(casadi::DM::vertcat({1.0, 5.0})));

    // parameter initialize
    opti_.set_value(_x_ref, casadi::DM::zeros(3, T_ + 1));
    opti_.set_value(_u_ref, casadi::DM::zeros(2, T_));

    u_init = casadi::DM::repmat({0,0}, 1, T_);
    x_init = casadi::DM::repmat({0.0, 0.0, 0.0}, 1, T_ + 1);
}

void NMPCController::initializeInHighCollision() {

    x_ = opti_.variable(nx_, T_ + 1);
    u_ = opti_.variable(nu_, T_);
    espsilon_ = opti_.variable(other_num);
    _x_ref = opti_.parameter(nx_, T_ + 1);
    _u_ref = opti_.parameter(nu_, T_);

    p_ = opti_.parameter(nx_, 1);
    Q = opti_.parameter(nx_, nx_);
    R = opti_.parameter(nu_, nu_);

    J_ = casadi::MX::dot(x_ - _x_ref, casadi::MX::mtimes(Q, x_ - _x_ref)) +
       casadi::MX::dot(u_ - _u_ref,casadi::MX::mtimes(R, u_ - _u_ref)) +
       10000*casadi::MX::dot(espsilon_, espsilon_) + // hard --> soft in collision avoidance
       1.5 * (u_(1) - 0.2); // deadlock heuristic method

    opti_.minimize(J_);

    // Define the control horizons
    for (int i = 0; i < T_; i++) {
        opti_.subject_to(x_(casadi::Slice(), i + 1) == x_(casadi::Slice(), casadi::Slice(i)) + \
                                                         dynamics_(x_(casadi::Slice(), casadi::Slice(i)), u_(casadi::Slice(), casadi::Slice(i)), dt_));
    }

    // constraint u and it derivative
    for (int i = 0; i < T_ - 1; i++) {
        auto devel = u_(casadi::Slice(), i + 1) - u_(casadi::Slice(), i) / dt_;
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_VELOCITY, devel(0), MAX_DELTA_VELOCITY));
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_OMEGA,devel(1), MAX_DELTA_OMEGA));
    }

    opti_.subject_to(casadi::Opti::bounded(0.0, u_(0), MAX_LINEAR_VELOCITY));
    opti_.subject_to(casadi::Opti::bounded(-MAX_ANGULAR_VELOCITY, u_(1), MAX_ANGULAR_VELOCITY));

    // constraint x
    opti_.subject_to(casadi::Opti::bounded(-M_PI, x_(2), M_PI));
    opti_.subject_to(x_(casadi::Slice(), 0) == p_);
    opti_.subject_to(casadi::Opti::bounded(-0.15, espsilon_, 0));

    //init other pos to (500,500)
    others_position=opti_.parameter(2, other_num);
    opti_.set_value(others_position, casadi::DM::repmat(std::vector<double>(2, 500), 1, other_num));

    // add collision free constraint
    for (int id = 0; id<other_num; id++)
        for (int i = 0; i < other_constraint_horizon; i++) {
            auto cur_other_dist = casadi::MX::norm_2(x_(casadi::Slice(0, 2), i) - others_position(casadi::Slice(), id));
            opti_.subject_to(cur_other_dist >= dist_other_th + espsilon_);
        }

    solver_options_["ipopt.print_level"] = 0;
    solver_options_["ipopt.sb"] = "yes";
    solver_options_["ipopt.max_iter"] = 2000;
    solver_options_["ipopt.tol"] = 1e-8;
    solver_options_["print_time"] = 0;
    solver_options_["ipopt.acceptable_obj_change_tol"] = 1e-6;
    opti_.solver("ipopt", solver_options_);

    // state and control weight
    opti_.set_value(Q,casadi::DM::diag(casadi::DM::vertcat({1.0, 1.0, 0.0})));
    opti_.set_value(R,casadi::DM::diag(casadi::DM::vertcat({1.0, 5.0})));

    // parameter initialize
    opti_.set_value(_x_ref, casadi::DM::zeros(3, T_ + 1));
    opti_.set_value(_u_ref, casadi::DM::zeros(2, T_));

    u_init = casadi::DM::repmat({0, 0}, 1, T_);
    x_init = casadi::DM::repmat({0.0, 0.0, 0.0},1,T_ + 1);
}

void NMPCController::setOthersState(Eigen::Vector2d pos, int id) {
    opti_.set_value(others_position(casadi::Slice(), id), std::vector<double>{pos.x(), pos.y()});
}

//set local goal position,yaw
void NMPCController::setReference(const std::vector<double> x_ref, const std::vector<double> u_ref) {
    opti_.set_value(_x_ref,casadi::DM::repmat(x_ref, 1, T_ + 1));
    opti_.set_value(_u_ref,casadi::DM::repmat(u_ref, 1, T_));
}

//update current state(x,y,yaw) and control(linear,angular velocity)
void NMPCController::setXUInit(const std::vector<double> x, const std::vector<double> u) {
    u_init=casadi::DM::repmat({u[0], u[1]}, 1, T_);
    x_init=casadi::DM::repmat({x[0], x[1], x[2]}, 1, T_ + 1);
    opti_.set_value(p_, x);
}

void NMPCController::solve() {
    opti_.set_initial(x_, x_init);
    opti_.set_initial(u_, u_init);

    casadi::OptiSol solution = opti_.solve();

    //get first control
    casadi::Matrix<double> u0 = solution.value(u_)(casadi::Slice(), 0);
    control0 = u0.get_elements();

    //get all state to show path
    casadi::Matrix<double> all_x = solution.value(x_);
    state = all_x.get_elements();

    // warm start
    // use first state and all control
    x_init = casadi::DM::repmat(solution.value(x_)(casadi::Slice(), 0), 1, T_ + 1);
    u_init = solution.value(u_)(casadi::Slice(), casadi::Slice());
}

casadi::MX NMPCController::dynamics_(const casadi::MX &x, const casadi::MX &u, const double dt) {
    casadi::MX xdot(nx_, 1);

    xdot(0) = u(0) * cos(x(2));
    xdot(1) = u(0) * sin(x(2));
    xdot(2) = u(1);
    return xdot * dt;
}

std::vector<double> NMPCController::getu() {
    return control0;
}

std::vector<double> NMPCController::getState() {
    return state;
}
