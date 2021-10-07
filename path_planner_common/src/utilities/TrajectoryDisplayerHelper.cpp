#include <path_planner_common/TrajectoryDisplayerHelper.h>
#include <stdio.h> // debug output

TrajectoryDisplayerHelper::TrajectoryDisplayerHelper(ros::NodeHandle& nodeHandle, ros::Publisher* displayPub, project11::Transformations &transformations, const std::string & map_frame): m_transformations(&transformations), m_map_frame(map_frame)
{
    m_display_pub = displayPub;
}

TrajectoryDisplayerHelper::TrajectoryDisplayerHelper(): m_transformations(nullptr) {
    m_display_pub = nullptr;
}

void TrajectoryDisplayerHelper::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) {
    displayTrajectory(trajectory, plannerTrajectory, true);
}

void TrajectoryDisplayerHelper::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory,
                                                  bool achievable) {
    // std::cerr << "DEBUG: TrajectoryDisplayerHelper::displayTrajectory() just called" << std::endl;
    if (!m_display_pub) throw std::runtime_error("Trajectory displayer not properly initialized");
    geographic_visualization_msgs::GeoVizPointList displayPoints;
    displayPoints.color.b = 1;
    if (!plannerTrajectory) {
        // controller trajectory
        displayPoints.color.a = 0.8;
        displayPoints.size = 10;
        if (achievable) {
            displayPoints.color.g = 1;
        } else {
            displayPoints.color.b = 0;
            displayPoints.color.r = 1;
        }
    } else {
        // std::cerr << "DEBUG: TrajectoryDisplayerHelper::displayTrajectory() received planner trajectory" << std::endl;
        // planner trajectory
        displayPoints.color.a = 1;
        displayPoints.size = 3.0;
        if (!achievable) {
            // dangerous, so color it red
            displayPoints.color.b = 0;
            displayPoints.color.r = 1;
        }
    }
    for (const State& s : trajectory) {
        // std::cerr << "DEBUG: TrajectoryDisplayerHelper::displayTrajectory() iterating through trajectory samples" << std::endl;
        geographic_msgs::GeoPoint point;
        displayPoints.points.push_back(convertToLatLong(s));
    }
    geographic_visualization_msgs::GeoVizItem geoVizItem;
    if (plannerTrajectory) {
        geoVizItem.id = "planner_trajectory";
    } else {
        geoVizItem.id = "controller_trajectory";
    }
    // std::cerr << "DEBUG: TrajectoryDisplayerHelper::displayTrajectory() set geoVizItem.id to " << geoVizItem.id << std::endl;
    // std::cerr << "DEBUG: displayPoints.size (unclear whether number of points or visual size of each point): " << displayPoints.size << std::endl;
    geoVizItem.lines.push_back(displayPoints);
    // std::cerr << "DEBUG: TrajectoryDisplayerHelper::displayTrajectory() geoVizItem.lines has this number of elements: " << geoVizItem.lines.size() << std::endl;
    m_display_pub->publish(geoVizItem);
    // std::cerr << "DEBUG: TrajectoryDisplayerHelper::displayTrajectory() just published geoVizItem via m_display_pub" << std::endl;
}

double TrajectoryDisplayerHelper::getTime() const {
    return ((double)ros::Time::now().toNSec()) / 1e9;
}

geographic_msgs::GeoPoint TrajectoryDisplayerHelper::convertToLatLong(const State& state) {
    if (!m_display_pub || !m_transformations) throw std::runtime_error("Trajectory displayer not properly initialized");
    
    geometry_msgs::Point point;
    
    point.x = state.x();
    point.y = state.y();
    
    return m_transformations->map_to_wgs84(point, m_map_frame);
}

path_planner_common::StateMsg TrajectoryDisplayerHelper::convertToStateMsg(const State& state) {
    if (!m_display_pub) throw std::runtime_error("Trajectory displayer not properly initialized");
    path_planner_common::StateMsg stateMsg;
    stateMsg.x = state.x();
    stateMsg.y = state.y();
    stateMsg.heading = state.heading();
    stateMsg.speed = state.speed();
    stateMsg.time = state.time();
    return stateMsg;
}

State TrajectoryDisplayerHelper::convertToStateFromMsg(const path_planner_common::StateMsg& stateMsg) {
    if (!m_display_pub) throw std::runtime_error("Trajectory displayer not properly initialized");
    State state;
    state.x() = stateMsg.x;
    state.y() = stateMsg.y;
    state.heading() = stateMsg.heading;
    state.speed() = stateMsg.speed;
    state.time() = stateMsg.time;
    return state;
}
