#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;

ActionModel::ActionModel(double k1_, double k2_) : last_pose{0ul, 0.0, 0.0, 0.0}, e1(0.0), e2(0.0), e3(0.0), dtheta(0.0), alpha(0.0), k1(k1_), k2(k2_)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (std::abs(odometry.x - last_pose.x) > .01 || std::abs(odometry.y - last_pose.y) > .01 || std::abs(odometry.theta - last_pose.theta) > .01) {
        double y_diff = odometry.y - last_pose.y;
        double x_diff = odometry.x - last_pose.x;
        double ds = std::sqrt(y_diff * y_diff + x_diff * x_diff);
        alpha = angle_diff(std::atan2(y_diff, x_diff), last_pose.theta);
        dtheta = angle_diff(odometry.theta, last_pose.theta);
        e1 = std::normal_distribution<>(0, k1 * std::abs(alpha));
        e2 = std::normal_distribution<>(ds, k2 * std::abs(ds));
        e3 = std::normal_distribution<>(0, k1 * std::abs(dtheta - alpha));
        last_pose = odometry;
        return true;
    }
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t updated = sample;
    updated.parent_pose = sample.pose;
    updated.pose.theta += alpha + e1(gen);
    updated.pose.utime = last_pose.utime;
    double ds = e2(gen);
    updated.pose.x += ds * std::cos(updated.pose.theta);
    updated.pose.y += ds * std::sin(updated.pose.theta);
    updated.pose.theta += dtheta - alpha + e3(gen);
    updated.pose.theta = wrap_to_2pi(updated.pose.theta);
    return updated;
}
