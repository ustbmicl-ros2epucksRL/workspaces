#include "path_searching/dyn_a_star.h"

#include <cmath>
#include <limits>

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            delete GridNodeMap_[i][j];
}


void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2d map_size)
{

    Eigen::Vector2d middle_pool_size=map_size/occ_map->getResolution();
    Eigen::Vector2i pool_size(middle_pool_size.x(),middle_pool_size.y());

    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr *[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr [POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNode;
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getDiagHeu2d(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    double h = 0.0;
    int diag = min(dx, dy);
    dx -= diag;
    dy -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(2.0) * diag + dy;
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(2.0) * diag + dx;
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    return dx + dy ;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::findNearestFreeIndex(const Vector2i &seed_idx, const Vector2i &reference_idx, Vector2i &free_idx, bool use_esdf_check) const
{
    if (!checkIndexBlocked(seed_idx, use_esdf_check))
    {
        free_idx = seed_idx;
        return true;
    }

    const int max_radius_cells = std::max(6, static_cast<int>(std::ceil(1.2 / step_size_)));
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int radius = 1; radius <= max_radius_cells; ++radius)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                if (std::max(std::abs(dx), std::abs(dy)) != radius)
                    continue;

                Vector2i candidate(seed_idx(0) + dx, seed_idx(1) + dy);
                if (candidate(0) < 1 || candidate(0) >= POOL_SIZE_(0) - 1 ||
                    candidate(1) < 1 || candidate(1) >= POOL_SIZE_(1) - 1)
                    continue;
                if (checkIndexBlocked(candidate, use_esdf_check))
                    continue;

                const double seed_dist = (candidate - seed_idx).cast<double>().norm();
                const double reference_dist = (candidate - reference_idx).cast<double>().norm();
                const double score = seed_dist + 0.05 * reference_dist;
                if (score < best_score)
                {
                    best_score = score;
                    free_idx = candidate;
                    found = true;
                }
            }
        }

        if (found)
            return true;
    }

    return false;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector2d start_pt, Vector2d end_pt, Vector2i &start_idx, Vector2i &end_idx, bool use_esdf_check)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    const Vector2i original_start_idx = start_idx;
    const Vector2i original_end_idx = end_idx;
    last_start_adjusted_ = false;
    last_end_adjusted_ = false;

    if (checkIndexBlocked(start_idx, use_esdf_check))
    {
        ROS_WARN_THROTTLE(1.0, "Start point is inside an obstacle; searching for a nearby free start.");
        if (!findNearestFreeIndex(original_start_idx, original_end_idx, start_idx, use_esdf_check))
        {
            ROS_WARN_THROTTLE(1.0, "Unable to recover occupied start point near %.2f %.2f.", start_pt.x(), start_pt.y());
            return false;
        }
        ROS_WARN_THROTTLE(
            1.0,
            "Recovered occupied start point to %.2f %.2f.",
            Index2Coord(start_idx).x(),
            Index2Coord(start_idx).y());
    }

    if (checkIndexBlocked(end_idx, use_esdf_check))
    {
        ROS_WARN_THROTTLE(1.0, "End point is inside an obstacle; searching for a nearby free end.");
        if (!findNearestFreeIndex(original_end_idx, start_idx, end_idx, use_esdf_check))
        {
            ROS_WARN_THROTTLE(1.0, "Unable to recover occupied end point near %.2f %.2f.", end_pt.x(), end_pt.y());
            return false;
        }
    }
    last_start_adjusted_ = (start_idx - original_start_idx).squaredNorm() != 0;
    last_end_adjusted_ = (end_idx - original_end_idx).squaredNorm() != 0;
    last_adjusted_start_ = Index2Coord(start_idx);
    last_adjusted_end_ = Index2Coord(end_idx);
    return true;
}

bool AStar::AstarSearch(const double step_size, Vector2d start_pt, Vector2d end_pt, bool use_esdf_check)
{
    ros::WallTime time_1 = ros::WallTime::now();
    ++rounds_;
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector2i start_idx, end_idx;
    gridPath_.clear();
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx, use_esdf_check))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1))
        {
            ros::WallTime time_2 = ros::WallTime::now();
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; 

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0 )
                        continue;

                    Vector2i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 )
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if(use_esdf_check){
                        if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                            continue;
                    } else {
                        if (checkOccupancy(Index2Coord(neighborPtr->index)))
                            continue;
                    }
                    
                    double static_cost = sqrt(dx * dx + dy * dy);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }

        ros::WallTime time_2 = ros::WallTime::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    ros::WallTime time_2 = ros::WallTime::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    //if planning failure, return one point that start point
    gridPath_= retrievePath(startPtr);
    ROS_WARN("[No result]:Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;
}

vector<Vector2d> AStar::getPath()
{
    vector<Vector2d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

vector<Vector2d> AStar::astarSearchAndGetSimplePath(const double step_size, Vector2d start_pt, Vector2d end_pt) {
    // call astar search and get the path

    ros::WallTime nowTime1 = ros::WallTime::now();

    vector<Vector2d> path;
    bool search_ok = AstarSearch(step_size, start_pt, end_pt, true);
    const bool esdf_end_adjusted = last_end_adjusted_;
    const Eigen::Vector2d esdf_adjusted_end = last_adjusted_end_;
    path = getPath();

    if (!search_ok || path.empty() || (path[0] - start_pt).norm() > 0.8) {
        ROS_WARN("I don't know why, but only try A* again");
        search_ok = AstarSearch(step_size, start_pt, end_pt, false);
        path = getPath();
    }

    if (!search_ok || path.empty()) {
        ROS_WARN_THROTTLE(1.0, "A* failed, falling back to a direct local segment.");
        path.clear();
        path.push_back(start_pt);
    }

    Eigen::Vector2d final_end = end_pt;
    if (checkOccupancy_esdf(end_pt) && esdf_end_adjusted) {
        final_end = esdf_adjusted_end;
    } else if (last_end_adjusted_) {
        final_end = last_adjusted_end_;
    }

    if ((final_end - end_pt).norm() > 1e-6) {
        ROS_WARN_THROTTLE(
            1.0,
            "Requested path end %.2f %.2f is occupied; using nearby free end %.2f %.2f.",
            end_pt.x(),
            end_pt.y(),
            final_end.x(),
            final_end.y());
    }

    if (path.empty()) {
        path.push_back(start_pt);
    }
    if ((path.back() - final_end).norm() > 1e-6) {
        path.push_back(final_end);
    } else if (path.size() == 1) {
        path.push_back(final_end);
    }
    return path;
}

vector<Vector2d> AStar::refinePath(const vector<Vector2d>& path)
{
    bool is_show_debug = false;
    // generate the simple path
    vector<Vector2d> simple_path;
    int size = path.size();
    if (size == 2){
        ROS_WARN("the path only have two points");
        return path;
    }
    else if(size ==1)
    {
        ROS_WARN("planning failed!!!");
        return path;
    }
        
    int end_idx   = 1;
    Vector2d cut_start = path[0];
    auto appendIfDistinct = [&simple_path](const Vector2d& point) {
        if (simple_path.empty() || (simple_path.back() - point).norm() > 1e-6)
            simple_path.push_back(point);
    };
    appendIfDistinct(cut_start);

    bool finish = false;
    int simplify_guard = 0;
    const int simplify_guard_limit = std::max(16, size * 2);
    while (!finish && simplify_guard++ < simplify_guard_limit) {
        bool made_progress = false;
        for (int i = end_idx; i < size; i++){
            bool is_safe = true;
            Vector2d check_pt = path[i];
            double pts_dis=(check_pt - cut_start).norm();
            int check_num = std::max(1, static_cast<int>(ceil(pts_dis / 0.01)));
            if(pts_dis>0.3)
                is_safe= false;
            // check collision
            for (int j=0; j<=check_num; j++){
                double alpha = double(1.0 / check_num) * j;
                Vector2d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;
                if (checkOccupancy_esdf(check_safe_pt)){
                    is_safe = false;
                    break;
                }
            }
            
            if (is_safe && i == (size -1)){
                finish = true;
                appendIfDistinct(check_pt);
                made_progress = true;
                break;
            }

            if (is_safe){
                continue;
            }
            else{
                if (i <= end_idx) {
                    cut_start = path[i];
                    end_idx = i + 1;
                    appendIfDistinct(cut_start);
                } else {
                    end_idx = i;
                    cut_start = path[end_idx-1];
                    appendIfDistinct(cut_start);
                }
                if (end_idx >= size) {
                    finish = true;
                }
                made_progress = true;
                break;
            }
        }
        if (!made_progress)
            break;
    }

    if (!finish) {
        ROS_WARN_THROTTLE(1.0,
                          "A* path refinement hit progress guard; keeping bounded refined path.");
        appendIfDistinct(path.back());
    }

    // debug
    if (is_show_debug){
        cout << "[simple A* path] : --------- " << endl;
        int n1 = simple_path.size();
        cout << "simple A* path size : " << n1 << endl;
        for (int i=0; i<n1; i++)
            cout << simple_path[i].transpose() << endl;
    }
    

    // check the near points and delete it
    bool near_flag;
    do
    {
        near_flag = false;
        if (simple_path.size() <=2){
            near_flag = false;
            break;
        }

        int num_same_check = simple_path.size();
        for (int i=0; i<num_same_check-1; i++){
            double len = (simple_path[i+1] - simple_path[i]).norm();
            if (len < 0.3){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }
        
    } while (near_flag);

    // debug
    if (is_show_debug){
        cout << "[delete simple path] : --------- " << endl;
        int n2 = simple_path.size();
        cout << "delete simple path size : " << n2 << endl;
        for (int i=0; i<n2; i++)
            cout << simple_path[i].transpose() << endl;
    }
    
    // check the path and add a point if two of them are too far away
    bool too_long_flag;
    const double length_threshold = 3;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            double leng = (simple_path[i+1] - simple_path[i]).norm();
            if (leng > length_threshold){
                Vector2d insert_point = (simple_path[i+1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin()+i+1 ,insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 10);

    // debug
    if (is_show_debug){
        cout << "[final simple path] : --------- " << endl;
        int n3 = simple_path.size();
        cout << "final simple path size : " << n3 << endl;
        for (int i=0; i<n3; i++)
            cout << simple_path[i].transpose() << endl;
    }

    ros::WallTime nowTime2=ros::WallTime::now();
//    ROS_WARN("[Planning total Time]:%f",(nowTime2-nowTime1).toSec());

    return simple_path;
}
