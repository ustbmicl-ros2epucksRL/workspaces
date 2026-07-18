#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include "plan_manager/clearance_grid.h"

namespace
{

nav_msgs::OccupancyGrid makeMap(int width, int height, double resolution = 1.0)
{
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.width = static_cast<std::uint32_t>(width);
    map.info.height = static_cast<std::uint32_t>(height);
    map.info.resolution = resolution;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<std::size_t>(width * height), 0);
    return map;
}

void occupy(nav_msgs::OccupancyGrid* map, int x, int y)
{
    map->data[static_cast<std::size_t>(y * static_cast<int>(map->info.width) + x)] = 100;
}

}  // namespace

TEST(ClearanceGrid, RejectsMalformedDataLength)
{
    nav_msgs::OccupancyGrid map = makeMap(4, 3);
    map.data.pop_back();

    const plan_manager::ClearanceGrid grid(map, true);

    EXPECT_FALSE(grid.valid());
    EXPECT_NE(grid.error().find("data length"), std::string::npos);
}

TEST(ClearanceGrid, ComputesSymmetricDistanceFromSingleObstacle)
{
    nav_msgs::OccupancyGrid map = makeMap(9, 9);
    occupy(&map, 4, 4);
    const plan_manager::ClearanceGrid grid(map, true);
    ASSERT_TRUE(grid.valid()) << grid.error();

    EXPECT_DOUBLE_EQ(grid.clearance(4, 4), 0.0);
    EXPECT_NEAR(grid.clearance(3, 4), grid.clearance(5, 4), 1e-12);
    EXPECT_NEAR(grid.clearance(4, 3), grid.clearance(4, 5), 1e-12);
    EXPECT_NEAR(grid.clearance(3, 3), grid.clearance(5, 5), 1e-12);
    EXPECT_GT(grid.clearance(2, 4), grid.clearance(3, 4));
}

TEST(ClearanceGrid, TreatsOutsideMapAsOccupiedBoundary)
{
    const nav_msgs::OccupancyGrid map = makeMap(7, 7);
    const plan_manager::ClearanceGrid grid(map, true);
    ASSERT_TRUE(grid.valid()) << grid.error();

    EXPECT_NEAR(grid.clearance(0, 3), 0.5, 1e-12);
    EXPECT_NEAR(grid.clearance(3, 3), 3.5, 1e-12);
    EXPECT_FALSE(grid.isFeasible(0, 3, 0.51));
    EXPECT_TRUE(grid.isFeasible(3, 3, 0.51));
}

TEST(ClearanceGrid, OverlaysCircularPhysicalObstaclesWithoutMutatingSource)
{
    nav_msgs::OccupancyGrid map = makeMap(8, 8, 0.5);
    map.info.origin.position.x = -1.0;
    map.info.origin.position.y = -1.0;
    const std::size_t original_occupied = static_cast<std::size_t>(1 * 8 + 1);
    map.data[original_occupied] = 100;

    const nav_msgs::OccupancyGrid overlay = plan_manager::overlayCircularObstacles(
        map, {{Eigen::Vector2d(1.0, 1.0), 0.40}});

    EXPECT_EQ(map.data[static_cast<std::size_t>(4 * 8 + 4)], 0)
        << "the source occupancy grid must remain immutable";
    EXPECT_EQ(overlay.data[original_occupied], 100)
        << "existing occupied cells must be preserved";
    EXPECT_EQ(overlay.data[static_cast<std::size_t>(4 * 8 + 4)], 100)
        << "cell center (1.25, 1.25) lies inside the physical obstacle";
    EXPECT_EQ(overlay.data[static_cast<std::size_t>(5 * 8 + 5)], 0)
        << "the overlay must not occupy cells outside the physical radius";
}

TEST(ClearanceGrid, CircularOverlayHonorsRotatedMapOrigin)
{
    nav_msgs::OccupancyGrid map = makeMap(5, 5, 1.0);
    const double half_sqrt_two = std::sqrt(0.5);
    map.info.origin.orientation.z = half_sqrt_two;
    map.info.origin.orientation.w = half_sqrt_two;
    // With a +90 degree origin yaw, map cell (1, 2) is centered at (-2.5, 1.5).
    const nav_msgs::OccupancyGrid overlay = plan_manager::overlayCircularObstacles(
        map, {{Eigen::Vector2d(-2.5, 1.5), 0.1}});

    EXPECT_EQ(overlay.data[static_cast<std::size_t>(2 * 5 + 1)], 100);
    EXPECT_EQ(overlay.data[static_cast<std::size_t>(1 * 5 + 2)], 0);
}
