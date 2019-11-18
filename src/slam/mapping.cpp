#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, last_pose{ 0, 0, 0, 0 }
{
}

static std::vector<Point<int>> bresenham(const Point<int>& start, const Point<int>& end, const OccupancyGrid& grid)
{
    std::vector<Point<int>> coords;
    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);
    int sx = start.x < end.x ? 1 : -1;
    int sy = start.y < end.y ? 1 : -1;
    int err = dx - dy;
    Point<int> current = start;

    while (grid.isCellInGrid(current.x, current.y) && (current.x != end.x || current.y != end.y))
    {
        coords.push_back(current);
        int e2 = 2 * err;
        if (e2 >= -dy)
        {
            err -= dy;
            current.x += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            current.y += sy;
        }
    }
    return coords;
}

void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (last_pose.utime == 0) {
        last_pose = pose;
        return;
    }
    MovingLaserScan lidar_rays = MovingLaserScan(scan, last_pose, pose);
    for (const adjusted_ray_t& ray : lidar_rays) {
        Point<float> vec { ray.range * std::cos(ray.theta), ray.range * std::sin(ray.theta) };
        Point<float> endpoint = ray.origin + vec;
        Point<int> ray_start_grid = global_position_to_grid_cell(ray.origin, map);
        Point<int> ray_end_grid = global_position_to_grid_cell(endpoint, map);
        for (const Point<int>& grid_cell : bresenham(ray_start_grid, ray_end_grid, map)) {
            map.setLogOdds(grid_cell.x, grid_cell.y, 127); // TODO not correct
        }
    }
    last_pose = pose;
}