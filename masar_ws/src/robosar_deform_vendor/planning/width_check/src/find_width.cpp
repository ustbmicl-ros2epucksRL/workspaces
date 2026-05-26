#include "width_check/find_width.h"

#include <algorithm>
#include <cmath>

void FindMaxFreeRegion::initMapandMaxGap(GridMap::Ptr occ_map_, double max_gap)
{
    grid_map_ = occ_map_;
    max_formation_gap = max_gap;
};

void FindMaxFreeRegion::updateFormationGap(double cur_gap) {
    max_formation_gap = cur_gap;
}

void FindMaxFreeRegion::setEndpointandupdateBoundary(Eigen::Vector2d odom, Eigen::Vector2d local_goal) {
    line_middle.setEndpoint(odom, local_goal);
    Eigen::Vector2d midpoint = (odom + local_goal) / 2;
    std::vector<Eigen::Vector2d> normal_pts;
    Eigen::Vector2d normal_vec(line_middle.getCoeff().x(), line_middle.getCoeff().y());
    normal_pts.resize(4);
    normal_pts[0] = odom + normal_vec / normal_vec.norm() * max_formation_gap / 2.0;
    normal_pts[1] = odom - normal_vec / normal_vec.norm() * max_formation_gap / 2.0;
    normal_pts[2] = local_goal + normal_vec / normal_vec.norm() * max_formation_gap / 2.0;
    normal_pts[3] = local_goal - normal_vec / normal_vec.norm() * max_formation_gap / 2.0;

    double min_x, min_y, max_x, max_y;
    double local_bound = sqrt(2) * max_formation_gap;
    min_x = midpoint.x() + local_bound;
    max_x = midpoint.x() - local_bound;
    min_y = midpoint.y() + local_bound;
    max_y = midpoint.y() - local_bound;

    for (int i = 0; i < normal_pts.size(); i++) {
        min_x = min(min_x, normal_pts[i].x());
        max_x = max(max_x, normal_pts[i].x());
        min_y = min(min_y, normal_pts[i].y());
        max_y = max(max_y, normal_pts[i].y());
    }
    boundary_min(0) = min_x;
    boundary_min(1) = min_y;
    boundary_max(0) = max_x;
    boundary_max(1) = max_y;
}

vector<Eigen::Vector2d> FindMaxFreeRegion::getBox() {
    std::vector<Eigen::Vector2d> tmp_box;
    Eigen::Vector2d box_pt;
    box_pt << boundary_min.x(), boundary_min.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_min.x(), boundary_max.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_max.x(), boundary_max.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_max.x(), boundary_min.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_min.x(), boundary_min.y();
    tmp_box.push_back(box_pt);
    return tmp_box;
}

bool FindMaxFreeRegion::updatePointsRegion() {
    grid_map_->getBoundaryMap(boundary_min, boundary_max, pts_all);

    pts_greater.clear();
    pts_less.clear();
    for (int pt_index = 0; pt_index < pts_all.size(); pt_index++)
    {
        int posInLine = line_middle.inLine(pts_all[pt_index]);
        if (posInLine == 0)
            return false;//obs in path
        else if (posInLine > 0)
            pts_greater.push_back(pts_all[pt_index]);
        else
            pts_less.push_back(pts_all[pt_index]);
    }
    return true;
}

void FindMaxFreeRegion::solve_max_b() {
    const Eigen::Vector3d middle_coeff = line_middle.getCoeff();
    const Eigen::Vector2d normal(middle_coeff.x(), middle_coeff.y());
    const double normal_norm = normal.norm();
    const double middle_b = middle_coeff.z();

    if (!std::isfinite(normal_norm) || normal_norm < 1e-9 ||
        !std::isfinite(middle_b) || !std::isfinite(max_formation_gap)) {
        line_greater.setCoeff(middle_coeff);
        line_less.setCoeff(middle_coeff);
        return;
    }

    const double max_offset = std::max(0.0, max_formation_gap * 0.5) * normal_norm;
    const double eps = std::max(1e-9, normal_norm * 1e-6);
    double lower_greater_b = middle_b - max_offset;
    double upper_less_b = middle_b + max_offset;

    for (const auto& pt : pts_greater) {
        lower_greater_b = std::max(lower_greater_b, -normal.dot(pt));
    }
    for (const auto& pt : pts_less) {
        upper_less_b = std::min(upper_less_b, -normal.dot(pt));
    }

    Eigen::Vector3d coeff_temp = middle_coeff;
    if (lower_greater_b >= middle_b - eps || upper_less_b <= middle_b + eps) {
        coeff_temp.z() = middle_b;
        line_greater.setCoeff(coeff_temp);
        line_less.setCoeff(coeff_temp);
        return;
    }

    coeff_temp.z() = lower_greater_b + eps;
    line_greater.setCoeff(coeff_temp);
    coeff_temp.z() = upper_less_b - eps;
    line_less.setCoeff(coeff_temp);
}

double FindMaxFreeRegion::getWidth() {
    double d_less = std::abs(line_middle.getb() - line_less.getb()) / line_middle.getNormal().norm();
    double d_greater = std::abs(line_middle.getb() - line_greater.getb()) / line_middle.getNormal().norm();
    width = d_less + d_greater;
    return width;
}

vector<Eigen::Vector2d> & FindMaxFreeRegion::getLocalMap() {
    return pts_all;
}

vector<Eigen::Vector2d> & FindMaxFreeRegion::getLocalMapGreater() {
    return pts_greater;
}

vector<Eigen::Vector2d> & FindMaxFreeRegion::getLocalMapLess() {
    return pts_less;
}

Eigen::Vector2d FindMaxFreeRegion::getWidthBestPoint() {
    double mid_b = (line_less.getb() + line_greater.getb()) * 0.5;
    LINE mid_line;
    Eigen::Vector3d coeff(line_middle.getCoeff());
    coeff.z() = mid_b;
    mid_line.setCoeff(coeff);

    double d_mid = std::abs(line_middle.getb() - mid_line.getb()) / line_middle.getNormal().norm();
    Eigen::Vector2d pt_temp = line_middle.getPoint_right() + line_middle.getNormal() / line_middle.getNormal().norm() * d_mid;
    if (mid_line.inLine(pt_temp) != 0)
        pt_temp = line_middle.getPoint_right() - line_middle.getNormal() / line_middle.getNormal().norm() * d_mid;

   return pt_temp;
}

Eigen::Vector2d FindMaxFreeRegion::getDirection() {
    return line_middle.getNormal();
}

void FindMaxFreeRegion::test(std::vector<Eigen::Vector2d>& one_line,std::vector<Eigen::Vector2d>& other_line)
{
    //greater
    double d_greater = std::abs(line_middle.getb() - line_greater.getb()) / line_middle.getNormal().norm();
    Eigen::Vector2d pt1_temp = line_middle.getPoint_left() + line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;
    if (line_greater.inLine(pt1_temp) != 0)
        pt1_temp = line_middle.getPoint_left() - line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;

    Eigen::Vector2d pt2_temp = line_middle.getPoint_right() + line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;

    if (line_greater.inLine(pt2_temp) != 0)
        pt2_temp = line_middle.getPoint_right() - line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;
    vector<Eigen::Vector2d> greater_end;
    greater_end.resize(2);
    greater_end[0] = pt1_temp;
    greater_end[1] = pt2_temp;

    //less
    double d_less = std::abs(line_middle.getb() - line_less.getb()) / line_middle.getNormal().norm();
    pt1_temp = line_middle.getPoint_left() + line_middle.getNormal() / line_middle.getNormal().norm() * d_less;
    if (line_less.inLine(pt1_temp) != 0)
        pt1_temp = line_middle.getPoint_left() - line_middle.getNormal() / line_middle.getNormal().norm() * d_less;

    pt2_temp = line_middle.getPoint_right() + line_middle.getNormal() / line_middle.getNormal().norm() * d_less;

    if (line_less.inLine(pt2_temp) != 0)
        pt2_temp = line_middle.getPoint_right() - line_middle.getNormal() / line_middle.getNormal().norm() * d_less;
    vector<Eigen::Vector2d> less_end;
    less_end.resize(2);
    less_end[0] = pt1_temp;
    less_end[1] = pt2_temp;

    one_line = greater_end;
    other_line = less_end;
}
