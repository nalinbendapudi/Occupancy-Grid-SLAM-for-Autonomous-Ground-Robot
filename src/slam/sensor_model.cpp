#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>

#include <iostream>
using namespace std;

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    double accumulated_odds = 0.0;

    // TODO make sure that if you, hypothetically, were to out-of-bounds this
    for (const adjusted_ray_t& ray : MovingLaserScan(scan, sample.parent_pose, sample.pose)) {
        Point<float> vec { ray.range * std::cos(ray.theta), ray.range * std::sin(ray.theta) };
        Point<float> endpoint = ray.origin + vec;
        Point<int> ray_direction {0, 0};
        if (std::abs(ray.theta) <= M_PI / 4.0) {
            ray_direction = {1, 0};
        } else if (ray.theta >= M_PI / 4.0 && ray.theta <= 3.0 * M_PI / 4.0) {
            ray_direction = {0, 1};
        } else if (std::abs(ray.theta) >= 3.0 * M_PI / 4.0) {
            ray_direction = {-1, 0};
        } else if (ray.theta <= -M_PI / 4.0 && ray.theta >= -3.0 * M_PI / 4.0) {
            ray_direction = {0, -1};
        } else {
            cout << "Ray direction invalid!" << endl;
        }
        Point<int> endpoint_grid = global_position_to_grid_cell(endpoint, map);
        if (!map.isCellInGrid(endpoint_grid.x, endpoint_grid.y)) {
            continue; // if the point on the map we hit is invalid, not too much we can say
        }
        int log_odds_at_endpoint = map.logOdds(endpoint_grid.x, endpoint_grid.y);
        if (log_odds_at_endpoint > 0) {
            // TODO should we enforce it "should" hit a closer cell?
            accumulated_odds += log_odds_at_endpoint;
        }
        else {
            Point<int> after_endpoint_grid = endpoint_grid + ray_direction;
            Point<int> before_endpoint_grid = endpoint_grid - ray_direction;
            int log_odds_after_endpoint = map.logOdds(after_endpoint_grid.x, after_endpoint_grid.y);
            int log_odds_before_endpoint = map.logOdds(before_endpoint_grid.x, before_endpoint_grid.y);
            if (log_odds_after_endpoint > 0) {
                accumulated_odds += log_odds_after_endpoint * fraction_for_adjacency;
            } else if (log_odds_before_endpoint > 0)
            {
                accumulated_odds += log_odds_before_endpoint * fraction_for_adjacency;
            }
            // log_odds_after_endpoint
        }
    }
    return accumulated_odds;
}
