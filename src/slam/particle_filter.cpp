#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>

#include <random>

using namespace std;

ParticleFilter::ParticleFilter(int numParticles, double k1, double k2)
    : kNumParticles_ (numParticles),
    posterior_(numParticles, particle_t {{0ul, 0.0, 0.0, 0.0}, {0ul, 0.0, 0.0, 0.0}, 1.0 / numParticles}),
    actionModel_(k1, k2),
    gen(rd()),
    resample_offset_sampler(0.0, 1.0 / numParticles)
{
    assert(kNumParticles_ > 1);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    for (auto& particle : posterior_) {
        particle.pose = pose;
    }
    posteriorPose_ = pose;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        posterior_ = computeProposalDistribution(prior);
        computeNormalizedPosterior(laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    vector<particle_t> prior;
    prior.reserve(kNumParticles_);
    // Select our random starting point
    double target_total_weight = resample_offset_sampler(gen);
    double current_weight = 0.0;

    auto current_particle = posterior_.cbegin();
    while (prior.size() < kNumParticles_) {
        while (current_particle + 1 != posterior_.cend() && current_weight < target_total_weight) {
            current_weight += current_particle->weight;
            ++current_particle;
            assert(current_particle != posterior_.cend());
        }
        prior.push_back(*current_particle);
        target_total_weight += 1.0 / kNumParticles_;
    }

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    proposal.reserve(kNumParticles_);
    for (const auto& particle : prior) {
        proposal.push_back(actionModel_.applyAction(particle));
    }

    return proposal;
}


void ParticleFilter::computeNormalizedPosterior(const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    double weight_sum = 0.0;
    for (auto& particle : posterior_) {
        particle.weight = sensorModel_.likelihood(particle, laser, map);
        weight_sum += particle.weight;
    }
    for (auto& particle : posterior_) {
        particle.weight /= weight_sum;
    }
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    Point<double> direction {0, 0};
    pose_xyt_t pose = posterior.front().pose;
    pose.x = 0;
    pose.y = 0;
    for (const auto& particle : posterior) {
        pose.x += particle.pose.x * particle.weight;
        pose.y += particle.pose.y * particle.weight;
        direction.x += std::cos(particle.pose.theta);
        direction.y += std::sin(particle.pose.theta);
    }
    pose.theta = std::atan2(direction.y, direction.x);

    return pose;
}
