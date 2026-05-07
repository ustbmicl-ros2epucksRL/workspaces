#ifndef RRT_H
#define RRT_H

#include <vector>
#include <queue>
#include <unordered_map>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "functions.h"
#include "Node.h"

// Coverage planner parameters
#define EPSILON 10e-5f
#define COV_MIN_INFO_GAIN_RADIUS_M 0.5
#define COV_PERCENT_COVERAGE_OVERLAP 0.75f        // TODO
#define COV_MIN_DIST_BETWEEN_COVERAGE_POINTS 1.5f // TODO
#define COV_MAX_INFO_GAIN_RADIUS_M 5.0            // Should reflect sensor model
std::vector<std::vector<int>> bfs_prop_model = {{-1, 0}, {0, -1}, {0, 1}, {1, 0}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

class RRT
{
public:
    RRT() : map_to_root_(){};
    RRT(geometry_msgs::Pose root_pose, float info_gain)
    {
        tf2::Quaternion quat_tf;
        tf2::convert(root_pose.orientation, quat_tf);
        tf2::Vector3 pos_tf;
        tf2::convert(root_pose.position, pos_tf);
        map_to_root_ = tf2::Transform(quat_tf, pos_tf);
        root_node_ = add_node(map_to_root_.getOrigin()[0], map_to_root_.getOrigin()[1], -1, info_gain);
        assert(root_node_);
        root_node_->set_root();
    };
    ~RRT(){};

    std::shared_ptr<Node> get_node(int id)
    {
        if (nodes_.find(id) == nodes_.end())
        {
            ROS_DEBUG("Node ID %d does not exist, cannot get.", id);
            return nullptr;
        }
        return nodes_[id];
    }

    std::shared_ptr<Node> add_node(float x, float y, int parent, float info_gain)
    {
        tf2::Transform root_to_node_tf = map_to_relative(x, y);
        nodes_[next_id_] = std::make_shared<Node>(x, y, root_to_node_tf, next_id_, parent, info_gain);
        auto parent_node = get_node(parent);
        if (parent_node)
            parent_node->add_child(next_id_);
        return nodes_[next_id_++];
    }

    void remove_node(int id)
    {
        if (nodes_.find(id) == nodes_.end())
        {
            ROS_WARN("Node ID %d does not exist, cannot remove.", id);
            return;
        }
        auto curr_node = get_node(id);
        std::unordered_set<int> children = curr_node->get_children();
        // A leaf node
        if (children.size() == 0)
        {
            if (!curr_node->is_root())
            {
                auto parent = get_node(curr_node->get_parent());
                parent->remove_child(id);
            }
            nodes_.erase(id);
            return;
        }
        // remove children
        for (int child_id : children)
        {
            remove_node(child_id);
        }
        remove_node(id);
    }

    void disable_node(int id)
    {
        ROS_DEBUG("disabling node %d", id);
        if (nodes_.find(id) == nodes_.end())
        {
            ROS_WARN("Node ID %d does not exist, cannot disable.", id);
            return;
        }
        auto curr_node = get_node(id);
        if (curr_node->is_disabled()) {
            ROS_WARN("Node ID %d is already disabled.", id);
            return;
        }
        disabled_nodes_.insert(id);
        std::unordered_set<int> active_children = curr_node->get_active_children();
        // A leaf node
        if (active_children.size() == 0)
        {
            if (!curr_node->is_root())
            {
                auto parent = get_node(curr_node->get_parent());
                parent->disable_child(id);
            }
            curr_node->set_disabled(true);
            return;
        }
        // remove children
        for (int child_id : active_children)
        {
            disable_node(child_id);
        }
        disable_node(id);
    }

    void enable_node(int id)  {
        auto curr_node = get_node(id);
        auto parent_node = get_node(curr_node->get_parent());
        if (parent_node->is_disabled()) {
            ROS_DEBUG("Cannot enable a node whose parent is also disabled.");
            return;
        }
        pop_disabled_node(id);
        curr_node->set_disabled(false);
        if (!curr_node->is_root()) {
            parent_node->enable_child(id);
        }
    }

    std::shared_ptr<Node> get_parent_node(std::shared_ptr<Node> child)
    {
        return get_node(child->get_parent());
    }

    std::unordered_set<int> get_disabled_node_ids() {
        return disabled_nodes_;
    }

    void pop_disabled_node(int id) {
        if (disabled_nodes_.find(id) == disabled_nodes_.end()) {
            ROS_DEBUG("Node %d is not disabled.", id);
            return;
        }
        disabled_nodes_.erase(id);
    }

    int nearest(float x, float y)
    {
        float min = std::numeric_limits<float>::max();
        int min_index = -1;
        float temp;
        // ROS_WARN("x: %f, y: %f", x, y);

        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {
            if (!j->second->is_disabled()){
                temp = Norm(j->second->get_x(), j->second->get_y(), x, y);
                if (temp <= min)
                {
                    min = temp;
                    min_index = j->first;
                }
            }
        }

        return min_index;
    }

    float dijkstra(int src, int dest)
    {
        using iPair = std::pair<float, int>;
        std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
        std::vector<float> dist(nodes_.size(), std::numeric_limits<float>::max());
        pq.push({0.0, src});
        dist[src] = 0.0;

        while (!pq.empty())
        {
            int node_id = pq.top().second;
            auto node = get_node(node_id);
            pq.pop();

            // found dest
            if (node_id == dest)
            {
                return dist[dest];
            }

            for (int neighbor : node->get_children())
            {
                float weight = Norm(node->get_x(), node->get_y(), get_node(neighbor)->get_x(), get_node(neighbor)->get_y());
                if (dist[neighbor] > dist[node_id] + weight)
                {
                    dist[neighbor] = dist[node_id] + weight;
                    pq.push({dist[neighbor], neighbor});
                }
            }
            if (!node->is_root())
            {
                int parent_id = node->get_parent();
                float weight = Norm(node->get_x(), node->get_y(), get_node(parent_id)->get_x(), get_node(parent_id)->get_y());
                if (dist[parent_id] > dist[node_id] + weight)
                {
                    dist[parent_id] = dist[node_id] + weight;
                    pq.push({dist[parent_id], parent_id});
                }
            }\
        }
        return dist[dest];
    }

    tf2::Transform map_to_relative(float x, float y)
    {
        tf2::Transform map_to_node = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(x, y, 0.0));
        tf2::Transform root_to_node = map_to_root_.inverse() * map_to_node;
        return root_to_node;
    }

    void update_node_map_coords(int node_id)
    {
        std::shared_ptr<Node> node = get_node(node_id);
        tf2::Transform root_to_node = node->get_rel_tf();
        tf2::Transform map_to_node = map_to_root_ * root_to_node;
        node->set_x(map_to_node.getOrigin()[0]);
        node->set_y(map_to_node.getOrigin()[1]);
    }

    void update_rrt(geometry_msgs::Pose root_pose_new)
    {
        tf2::Quaternion quat_tf;
        tf2::convert(root_pose_new.orientation, quat_tf);
        tf2::Vector3 pos_tf;
        tf2::convert(root_pose_new.position, pos_tf);
        map_to_root_ = tf2::Transform(quat_tf, pos_tf);

        // traverse tree and update node map coords
        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {
            update_node_map_coords(j->first);
        }

        return;
    }

    void update_coverage_nodes(void) {

        coverage_nodes_.clear();
        // Mark all nodes as coverage nodes unless disabled
        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {
            j->second->is_coverage_node_ = true;
        }

        // Iterate through all nodes and mark nodes that are not coverage nodes
        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {
            // if node is already marked as not a coverage node, skip
           if(j->second->is_coverage_node_) {
                // Check if valid coverage point
                std::pair<float, float> node_pos = std::make_pair(j->second->get_x(), j->second->get_y());
                if (j->second->is_allocated_ || j->second->is_visited_ || (!j->second->is_disabled() &&
                        is_valid_coverage_node(node_pos, j->second->get_info_gain_radius(), j->second->get_id()))) 
                    {
                        j->second->is_coverage_node_ = true;
                        coverage_nodes_.push_back(j->second->get_id());
                        // ROS_INFO("Vertex %d is a coverage point", vertex.id_);
                    }
                    else
                    {
                        j->second->is_coverage_node_ = false;
                    }
            }
        }
    }

     /**** ====================== Naive filtering : remove coverage points that are too close to each other ======== */
    void filter_coverage_nodes(void) {

        coverage_nodes_.clear();
        for (auto j = nodes_.begin(); j != nodes_.end(); j++) {
           
            if (j->second->is_coverage_node_)
            {
                std::pair<float, float> node_pos = std::make_pair(j->second->get_x(), j->second->get_y());
                // A visited or allocated node is always a coverage node
                // Check if node passes filter
                if(j->second->is_visited_ || j->second->is_allocated_
                 || filter_node_on_threshold(node_pos, j->second->get_info_gain_radius(), j->second->get_id())) {
                    
                    coverage_nodes_.push_back(j->second->get_id());
                }
               
            }
        }

    }

    std::vector<int> coverage_nodes_;
    std::unordered_map<int, std::shared_ptr<Node>> nodes_;
    int next_id_ = 0;

private:

    bool is_valid_coverage_node(std::pair<float, float> x_new, float info_radius, int id)
    {

        // Local variables
        bool valid_node = true;
        std::vector<int> to_remove_ids;

        // parameter check
        if (info_radius < COV_MIN_INFO_GAIN_RADIUS_M)
        {
            return false;
        }

        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {

            // Dont compare with yourself
            if (j->second->get_id() == id)
            {
                continue;
            }

            // eliminate node which is more than info_radius away from x_new
            float max_info_radius = std::max(info_radius, j->second->get_info_gain_radius());
            if (fabs(x_new.first - j->second->get_x()) > max_info_radius || fabs(x_new.second - j->second->get_y()) > max_info_radius)
            {
                continue;
            }

            // TODO should we check if it is a coverage node?
            float inter_node_dist = Norm(j->second->get_x(), j->second->get_y(),
                                        x_new.first, x_new.second);
            // If there is an overlap (either of the centers is inside the other circle)
            // keep the node with the largest radius
            // TODO can control per cent overlap
            if ((info_radius > inter_node_dist || j->second->get_info_gain_radius() > inter_node_dist))
            {

                if (j->second->is_visited_ || j->second->is_allocated_)
                {
                    valid_node = false;
                    break;
                }

                if (info_radius < j->second->get_info_gain_radius())
                {

                    // std::cout<<j->get_info_gain_radius()<<" "<<info_radius<<" "<<" "<<inter_node_dist<<" "<<id<<" "<<j->id_<<std::endl;
                    valid_node = false;
                    break;
                }
                else
                {
                    to_remove_ids.push_back(j->second->get_id());
                }
            }

        }

        if (valid_node)
        {
            for (int id : to_remove_ids)
            {
                get_node(id)->is_coverage_node_ = false;
            }
        }
        return valid_node;
    }


bool filter_node_on_threshold(std::pair<float, float> x_new, float info_radius, int id)
{

    bool result = true;
    for (auto j = nodes_.begin(); j != nodes_.end(); j++) {

        if (j->second->get_id() == id)
        {
            continue;
        }
        if (j->second->is_coverage_node_)
        {
            float inter_node_dist = Norm(j->second->get_x(), j->second->get_y(),
                                        x_new.first, x_new.second);
            if (inter_node_dist < COV_MIN_DIST_BETWEEN_COVERAGE_POINTS) {

                // filter out smaller coverage node
                if (j->second->is_allocated_ || j->second->is_visited_ || info_radius < j->second->get_info_gain_radius())
                {
                    get_node(id)->is_coverage_node_ = false;
                    result = false;
                    break;
                }
                else
                {
                    j->second->is_coverage_node_ = false;
                }
            }
        }
        

    }
    return result;
}


    tf2::Transform map_to_root_;
    std::shared_ptr<Node> root_node_ = NULL;
    std::unordered_set<int> disabled_nodes_;
};

#endif // RRT_H