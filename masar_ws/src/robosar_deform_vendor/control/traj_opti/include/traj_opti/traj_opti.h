#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>

#define MAX_LINEAR_VELOCITY     0.22   // m/s
#define MAX_ANGULAR_VELOCITY    2.0   // rad/s
#define MAX_DELTA_VELOCITY    1.7   // m/s
#define MAX_DELTA_OMEGA    M_PI / 0.5   // rad/s

class NMPCController
{
public:
    NMPCController();
    NMPCController(ros::NodeHandle& nh_);
    void initialize();
    void initializeInHighCollision();
    void setReference(const std::vector<double> x_ref, const std::vector<double> u_ref);
    void setXUInit(const std::vector<double> x, const std::vector<double> u);
    void setOthersState(Eigen::Vector2d pos, int id);
    void solve();
    std::vector<double> getu();
    std::vector<double> getState();

    typedef std::shared_ptr<NMPCController> Ptr;
private:
    enum ProblemMode
    {
        UNINITIALIZED_MODE = 0,
        NORMAL_PROBLEM_MODE,
        HIGH_COLLISION_PROBLEM_MODE
    };

    casadi::MX dynamics_(const casadi::MX& x,const casadi::MX& u,const double dt);
    void rebuildProblem();

private:
    int T_; // horizon
    int nx_; // state size
    int nu_; // control size
    double dt_; // sampling time
    int other_num;
    double dist_other_th;
    int other_constraint_horizon;
    int rebuild_interval;
    int solve_count;
    int ipopt_max_iter;
    double ipopt_max_cpu_time;
    ProblemMode problem_mode;

    casadi::Opti opti_;
    casadi::MX Q;
    casadi::MX R;
    casadi::MX p_;
    casadi::MX x_;
    casadi::MX _x_ref;
    casadi::DM x_init;
    casadi::MX u_;
    casadi::MX _u_ref;
    casadi::DM u_init;
    casadi::MX J_;
    casadi::Dict solver_options_;
    casadi::MX espsilon_;
    casadi::MX others_position;

    std::vector<double> control0;
    std::vector<double> state;
    std::vector<double> last_x_ref;
    std::vector<double> last_u_ref;
    std::vector<double> last_x_init_seed;
    std::vector<double> last_u_init_seed;
    std::vector<Eigen::Vector2d> last_other_positions;
};

class TrajectoryOptimizer
{
public:
    TrajectoryOptimizer();
    explicit TrajectoryOptimizer(ros::NodeHandle& nh_);
    void setLocalGoal(Eigen::Vector2d local_goal, Eigen::Vector2d desired_wv);
    bool safeSetLocalGoal(Eigen::Vector2d local_goal, Eigen::Vector2d desired_wv);
    void setGlobalGoal(Eigen::Vector2d glb_goal);
    void updateState(Eigen::Vector3d s, Eigen::Vector2d c);
    void updateOtherOdom(Eigen::Vector2d pos, int id);
    bool run();
    std::vector<double> getControlVelocity();
    std::vector<double> getState();
    std::vector<double> fallbackVelocity() const;
    bool isFiniteVector(const Eigen::Vector2d& value) const;
    bool isFiniteVector(const Eigen::Vector3d& value) const;
    bool isFiniteVector(const std::vector<double>& value, std::size_t expected_size) const;
    void toggleCostFunc(int costFuncType);

    typedef std::shared_ptr<TrajectoryOptimizer> Ptr;
private:

    Eigen::Vector3d state; // posX, posY, Yaw
    Eigen::Vector2d goal;
    Eigen::Vector2d global_goal;
    std::vector<double> x_ref, u_ref;
    std::vector<double> vel;
    std::vector<Eigen::Vector2d> other_positions_;

    double dist_thresh{};
    bool stall_fallback_enabled{false};
    double stall_fallback_goal_distance{0.35};
    double stall_fallback_linear_epsilon{0.015};
    double fallback_other_clearance{0.4};
    double fallback_repulsion_gain{1.0};
    bool nmpc_enabled{true};
    bool nmpc_memory_guard_enabled{true};
    bool nmpc_memory_guard_triggered{false};
    double nmpc_rss_limit_mb{384.0};

    enum COST_FUNCTION_TYPE
    {
        NORMAL_MODE=0,
        HIGH_COLLISION_MODE
    };
    COST_FUNCTION_TYPE cur_cost_type;
    NMPCController::Ptr nmpc_;
};
