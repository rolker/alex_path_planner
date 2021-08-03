#include "GaussianDynamicObstaclesManager.h"

double GaussianDynamicObstaclesManager::collisionExists(double x, double y, double time, bool strict) const {
    double sum = 0;
    for (auto o : m_Obstacles) {
        auto& obstacle = o.second;
        obstacle.project(time);
        sum += obstacle.pdf(Eigen::Vector2d(x, y));
    }
    // questionable
    if (sum < 1e-5) return 0;
    return sum;
}

void GaussianDynamicObstaclesManager::update(uint32_t mmsi, double x, double y, double heading, double speed,
                                             double time) {
    if (!isIgnored(mmsi)) {
        // I guess the piecewise_construct / forward_as_tuple idiom here helps distinguish between key and value components of the map entry
        auto result = m_Obstacles.emplace(std::piecewise_construct,
                std::forward_as_tuple(mmsi),
                std::forward_as_tuple(x, y, heading, speed, time));
        if (!result.second) {
            result.first->second = Obstacle(x, y, heading, speed, time);
        }
    }
}

void GaussianDynamicObstaclesManager::forget(uint32_t mmsi) {
    m_Obstacles.erase(mmsi);
}

const std::unordered_map<uint32_t, GaussianDynamicObstaclesManager::Obstacle>& GaussianDynamicObstaclesManager::get() const {
    return m_Obstacles;
}

const std::unordered_map<uint32_t, GaussianDynamicObstaclesManager::Obstacle> GaussianDynamicObstaclesManager::get_deep_copy() const {
    // declare new map to be returned
    std::unordered_map<uint32_t, Obstacle> deep_copy;
    for (const auto & entry : m_Obstacles) {
        // deep copy the obstacle
        Obstacle obstacle_copy = Obstacle(entry.second);
        // construct map entry
        std::pair<std::uint32_t, Obstacle> new_entry (entry.first, obstacle_copy);
        // insert new entry into map
        deep_copy.insert(new_entry);
    }
    // returned map containing deep copies of obstacles
    return deep_copy;
}

void GaussianDynamicObstaclesManager::update(uint32_t mmsi, double x, double y, double heading, double speed,
                                             double time, Eigen::Matrix<double, 2, 2> covariance) {
    if (!isIgnored(mmsi)) {
        auto result = m_Obstacles.emplace(std::piecewise_construct,
                                          std::forward_as_tuple(mmsi),
                                          std::forward_as_tuple(x, y, heading, speed, time, covariance));
        if (!result.second) {
            result.first->second = Obstacle(x, y, heading, speed, time, covariance);
        }
    }
}
