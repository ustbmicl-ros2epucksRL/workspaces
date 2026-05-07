#include <gtest/gtest.h>
#include <climits>
#include "task_graph.hpp"

// bad function:
// for example: how to deal with overflow?
int add(int a, int b){
    return a + b;
}


// returns true if coverage points are redundant
bool intertreeCoverageFilter(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2)
{

  // check overlap
  // TODO should we check if it is a coverage node?
  float inter_node_dist = Norm(node1->get_x(), node1->get_y(),
                               node2->get_x(), node2->get_y());
  // If there is an overlap (either of the centers is inside the other circle)
  if ((node1->get_info_gain_radius() > inter_node_dist || node2->get_info_gain_radius() > inter_node_dist))
  {
    return true;
  }

  // check node to node distance
  if (inter_node_dist < COV_MIN_DIST_BETWEEN_COVERAGE_POINTS)
    return true;

  return false;
}

void intertreeCoverageFiltering(TaskGraph::TaskVertex *vertexPtr, std::vector<TaskGraph::TaskVertex>& V_)
{

  // Check if all coverage nodes in this tree are valid
  // if not remove them
  for (auto my_node_id_ptr = vertexPtr->rrt_.coverage_nodes_.begin();
       my_node_id_ptr != vertexPtr->rrt_.coverage_nodes_.end();)
  {

    bool redundant = false;
    auto my_node_ptr = vertexPtr->rrt_.get_node(*my_node_id_ptr);

    // dont filter allocated or visited coverage points
    if (my_node_ptr->is_allocated_ || my_node_ptr->is_visited_)
    {
      my_node_id_ptr++;
      continue;
    }

    for (auto j = V_.begin(); j != V_.end(); j++)
    {
      // dont compare with same tree
      if (j->id_ == vertexPtr->id_)
        continue;

      for (auto other_node_id_ptr = j->rrt_.coverage_nodes_.begin();
           other_node_id_ptr != j->rrt_.coverage_nodes_.end();)
      {
        auto other_node_ptr = j->rrt_.get_node(*other_node_id_ptr);

        if (intertreeCoverageFilter(my_node_ptr, other_node_ptr))
        {

          // filter out smaller coverage node
          if (other_node_ptr->is_allocated_ || other_node_ptr->is_visited_ || my_node_ptr->get_info_gain_radius() < other_node_ptr->get_info_gain_radius())
          {
            redundant = true;
            break;
          }
          else
          {
            other_node_ptr->is_coverage_node_ = false;
            other_node_id_ptr = j->rrt_.coverage_nodes_.erase(other_node_id_ptr);
            continue;
          }
        }

        other_node_id_ptr++;
      }

      if (redundant)
        break;
    }

    // delete if redundant
    if (redundant)
    {
      my_node_ptr->is_coverage_node_ = false;
      my_node_id_ptr = vertexPtr->rrt_.coverage_nodes_.erase(my_node_id_ptr);
    }
    else
    {
      my_node_id_ptr++;
    }
  }
}



TEST(TaskVertexUpdateTest, TaskVertexUpdateTest){
    //TaskGraph task_graph_session(true);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    TaskGraph::TaskVertex task_vertex(1, pose, 1.0);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex.rrt_.coverage_nodes_.size());

    // Add a smaller coverage node
    task_vertex.rrt_.add_node(0.1,0.1,1,0.5);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex.rrt_.coverage_nodes_.size());

    // `Add a larger coverage node
    task_vertex.rrt_.add_node(0.2,0.2,1,1.5);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex.rrt_.coverage_nodes_.size());
    ASSERT_EQ(task_vertex.rrt_.coverage_nodes_[0],2);

    // Add another coverage node
    task_vertex.rrt_.add_node(3,3,1,1.5);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(2, task_vertex.rrt_.coverage_nodes_.size());
    ASSERT_EQ(task_vertex.rrt_.coverage_nodes_[0],3);
    ASSERT_EQ(task_vertex.rrt_.coverage_nodes_[1],2);
}

TEST(TaskVertexFilterTest, TaskVertexFilterTest){
    //TaskGraph task_graph_session(true);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    TaskGraph::TaskVertex task_vertex(1, pose, 0.7);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex.rrt_.coverage_nodes_.size());

    // Add a coverage node inside filter
    task_vertex.rrt_.add_node(COV_MIN_DIST_BETWEEN_COVERAGE_POINTS-0.2,0.0,1,0.6);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(2, task_vertex.rrt_.coverage_nodes_.size());
    task_vertex.rrt_.filter_coverage_nodes();
    ASSERT_EQ(1, task_vertex.rrt_.coverage_nodes_.size());
    ASSERT_EQ(task_vertex.rrt_.coverage_nodes_[0],0);

     // Add a coverage node outside filter
    task_vertex.rrt_.add_node(COV_MIN_DIST_BETWEEN_COVERAGE_POINTS+0.2,0.0,1,0.6);
    task_vertex.rrt_.update_coverage_nodes();
    ASSERT_EQ(2, task_vertex.rrt_.coverage_nodes_.size());
    task_vertex.rrt_.filter_coverage_nodes();
    ASSERT_EQ(2, task_vertex.rrt_.coverage_nodes_.size());

}

TEST(InterTreeFilterConditionTest, InterTreeFilterConditionTest) {

  // Create rrt nodes
  auto node1_ptr = std::make_shared<Node>(0.0, 0.0, tf2::Transform()
                              , 1, 0, 1.0);
  auto node2_ptr = std::make_shared<Node>(0.0, 0.0, tf2::Transform()
                              , 1, 0, 0.8);

  ASSERT_TRUE(intertreeCoverageFilter(node1_ptr, node2_ptr));

  auto node3_ptr = std::make_shared<Node>(10.0, 10.0, tf2::Transform()
                              , 1, 0, 0.8);
  ASSERT_FALSE(intertreeCoverageFilter(node1_ptr, node3_ptr));

  auto node4_ptr = std::make_shared<Node>(0.0, COV_MIN_DIST_BETWEEN_COVERAGE_POINTS+0.1, tf2::Transform()
                              , 1, 0, 1.0);
  ASSERT_FALSE(intertreeCoverageFilter(node1_ptr, node4_ptr));

  auto node5_ptr = std::make_shared<Node>(0.0, COV_MIN_DIST_BETWEEN_COVERAGE_POINTS-0.1, tf2::Transform()
                              , 1, 0, 1.0);
  ASSERT_TRUE(intertreeCoverageFilter(node1_ptr, node5_ptr));
}


TEST(InterTreeFilterTest, InterTreeFilterTest){
    //TaskGraph task_graph_session(true);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;

    // Initialise vertices
    TaskGraph::TaskVertex task_vertex1(1, pose, 0.7);
    task_vertex1.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex1.rrt_.coverage_nodes_.size());

    TaskGraph::TaskVertex task_vertex2(2, pose, 0.6);
    task_vertex2.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex2.rrt_.coverage_nodes_.size());

    pose.position.x = 10.0;
    pose.position.y = 10.0;
    TaskGraph::TaskVertex task_vertex3(3, pose, 0.7);
    task_vertex3.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex3.rrt_.coverage_nodes_.size());

    // Invoke filter
    std::vector<TaskGraph::TaskVertex> V_;
    V_.push_back(task_vertex2);
    V_.push_back(task_vertex3);
    intertreeCoverageFiltering(&task_vertex1, V_);

    // Check results
    ASSERT_EQ(1, task_vertex1.rrt_.coverage_nodes_.size());
    ASSERT_EQ(0, V_[0].rrt_.coverage_nodes_.size());
    ASSERT_EQ(1, V_[1].rrt_.coverage_nodes_.size());

    // Larger vertex
    pose.position.x = 0.1;
    pose.position.y = 0.1;
    TaskGraph::TaskVertex task_vertex4(4, pose, 1.0);
    task_vertex4.rrt_.update_coverage_nodes();
    ASSERT_EQ(1, task_vertex4.rrt_.coverage_nodes_.size());
    V_.push_back(task_vertex4);
    intertreeCoverageFiltering(&task_vertex1, V_);

    // Check results
    ASSERT_EQ(0, task_vertex1.rrt_.coverage_nodes_.size());
    ASSERT_EQ(0, V_[0].rrt_.coverage_nodes_.size());
    ASSERT_EQ(1, V_[1].rrt_.coverage_nodes_.size());
    ASSERT_EQ(1, V_[2].rrt_.coverage_nodes_.size());
 
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}