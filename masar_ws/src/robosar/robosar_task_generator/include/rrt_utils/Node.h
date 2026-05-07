#ifndef NODE_H
#define NODE_H

#include <unordered_set>
#include <tf2/LinearMath/Transform.h>

class Node
{
public:
    Node(float x, float y, tf2::Transform root_to_node, int id, int parent_id, float info_gain)
        : x_(x), y_(y), root_to_node_(root_to_node), id_(id), parent_(parent_id), info_gain_radius_(info_gain),
          is_coverage_node_(false), is_allocated_(false), is_visited_(false) {};
    ~Node(){};

    bool is_root() { return is_root_; }
    bool is_disabled() {return disabled_;}
    void set_root() { is_root_ = true; }
    void set_disabled(bool disabled) {
        ROS_DEBUG("setting %d to disabled", id_);
        if (disabled && is_root_) {
            ROS_WARN("Cannot disable root node.");
            return;
        }
        disabled_ = disabled;
    }
    float get_x() { return x_; }
    float get_y() { return y_; }
    void set_x(float x) { x_ = x; }
    void set_y(float y) { y_ = y; }
    float get_rel_x() { return root_to_node_.getOrigin()[0]; }
    float get_rel_y() { return root_to_node_.getOrigin()[1]; }
    tf2::Transform get_rel_tf() {return root_to_node_;}
    int get_id() { return id_; }
    std::pair<float, float> get_coord() { return std::make_pair(x_, y_); }
    float get_info_gain_radius() { return info_gain_radius_; }

    void add_child(int child)
    {
        if (disabled_) {
            ROS_WARN("Cannot add child to a disabled node.");
            return;
        }
        children_.insert(child);
        active_children_.insert(child);
    }
    void remove_child(int child)
    {
        if (children_.find(child) == children_.end())
        {
            ROS_WARN("Child %d does not exist, cannot remove.", child);
            return;
        }
        children_.erase(child);
    }
    void disable_child(int child) {
        if (active_children_.find(child) == active_children_.end())
        {
            ROS_WARN("Child %d is already disabled.", child);
            return;
        }
        active_children_.erase(child);
    }
    void enable_child(int child) {
        if (children_.find(child) == children_.end())
        {
            ROS_WARN("Child %d does not exist.", child);
            return;
        }
        active_children_.insert(child);
    }
    std::unordered_set<int> get_children() { return children_; }
    std::unordered_set<int> get_active_children() { return active_children_;}
    int get_parent() { return parent_; }

    bool is_coverage_node_;
    bool is_visited_;
    bool is_allocated_;

private:
    float info_gain_radius_;
    float x_;
    float y_;
    const tf2::Transform root_to_node_;
    int id_;
    const int parent_;
    bool is_root_ = false;
    bool disabled_ = false;
    std::unordered_set<int> children_;
    std::unordered_set<int> active_children_;
};

#endif // NODE_H