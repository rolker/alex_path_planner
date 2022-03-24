#include <project11_navigation/interfaces/task_to_task_workflow.h>
#include "executive/executive.h"
#include "trajectory_publisher.h"
#include <alex_path_planner_common/Stats.h>
#include <alex_path_planner_common/TaskLevelStats.h>
#include <alex_path_planner_common/TrajectoryDisplayerHelper.h>
#include <alex_path_planner/alex_path_plannerConfig.h>
#include "project11/utils.h"
#include "project11/tf2_utils.h"
#include <pluginlib/class_list_macros.h>
#include "common/map/Costmap2DMap.h"

namespace alex_path_planner
{

struct SampleContext
{
  std::vector<geometry_msgs::PoseStamped>* pose_vector;
  ros::Time start_time;
  double speed;
  std::string frame_id;
};

/// Callback used to convert a curve to segments
int buildPath(double q[3], double t, void* user_data)
{
  SampleContext* c = reinterpret_cast<SampleContext*>(user_data);
    
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = c->frame_id;
  tf2::Quaternion quat(tf2::Vector3(0,0,1), q[2]);
  pose.pose.orientation = tf2::toMsg(quat);
    
  pose.pose.position.x = q[0];
  pose.pose.position.y = q[1];

  pose.header.stamp  = c->start_time + ros::Duration(t/c->speed);
  c->pose_vector->push_back(pose);
   
  return 0;
}


class AlexPathPlanner: public project11_navigation::TaskToTaskWorkflow, public TrajectoryPublisher
{
public:
  AlexPathPlanner()
  {
    executive_ = std::make_shared<Executive>(this);
  }

  ~AlexPathPlanner()
  {
    executive_.reset();
    std::cerr << strerror(errno) << std::endl;
  }

  void configure(std::string name, project11_navigation::Context::Ptr context) override
  {
    ROS_INFO_STREAM("Initializing AlexPathPlanner plugin with name " << name);

    context_ = context;

    ros::NodeHandle nh("~/" + name);
    transformations_ = std::make_shared<project11::Transformations>(&context_->tfBuffer());

    nh.param("step_size", step_size_, step_size_);

    nh.param("output_task_type", output_task_type_, output_task_type_);
    nh.param("output_task_name", output_task_name_, output_task_name_);

    nh.param("turning_radius", turning_radius_, turning_radius_);
    nh.param("coverage_turning_radius", coverage_turning_radius_, coverage_turning_radius_);
    nh.param("max_speed", max_speed_, max_speed_);
    nh.param("slow_speed", slow_speed_, slow_speed_);
    nh.param("line_width", line_width_, line_width_);
    nh.param("branching_factor", branching_factor_, branching_factor_);
    nh.param("time_horizon", time_horizon_, time_horizon_);
    nh.param("time_minimum", time_minimum_, time_minimum_);
    nh.param("collision_checking_increment", collision_checking_increment_, collision_checking_increment_);
    nh.param("initial_samples", initial_samples_, initial_samples_);
    nh.param("use_brown_paths", use_brown_paths_, use_brown_paths_);
    nh.param("heuristic", heuristic_, heuristic_);
    nh.param("gaussian_dynamic_obstacles", gaussian_dynamic_obstacles_, gaussian_dynamic_obstacles_);
    nh.param("ignore_dynamic_obstacles", ignore_dynamic_obstacles_, ignore_dynamic_obstacles_);
    nh.param("planner", planner_, planner_);

    stats_pub_ = nh.advertise<alex_path_planner_common::Stats>("stats", 1);
    task_level_stats_pub_ = nh.advertise<alex_path_planner_common::TaskLevelStats>("task_level_stats", 1);
    display_pub_ = nh.advertise<geographic_visualization_msgs::GeoVizItem>("display",1);
  }

  void setGoal(const std::shared_ptr<project11_navigation::Task>& input) override
  {
    input_task_ = input;
    output_task_.reset();
    executive_->cancelPlanner();
    executive_->clearRibbons();
    auto c = context_->costmap();
    if(c)
      executive_->setMap(std::make_shared<Costmap2DMap>(c));
    if(input_task_)
    {
      auto msg = input_task_->message();
      for(int i = 0; i+1 < msg.poses.size(); i++)
      {
        auto p1 = msg.poses[i];
        auto p2 = msg.poses[i+1];
        executive_->addRibbon(p1.pose.position.x, p1.pose.position.y, p2.pose.position.x, p2.pose.position.y);
      }

      auto caps = context_->getRobotCapabilities();
      //todo use caps

      auto data = input_task_->data();
      double turning_radius = turning_radius_;
      if(data["turning_radius"])
        turning_radius = data["turning_radius"].as<double>();
      double coverage_turning_radius = coverage_turning_radius_;
      if(data["coverage_turning_radius"])
        coverage_turning_radius = data["coverage_turning_radius"].as<double>();
      double max_speed = max_speed_;
      if(data["max_speed"])
        max_speed = data["max_speed"].as<double>();
      double slow_speed = slow_speed_;
      if(data["slow_speed"])
        slow_speed = data["slow_speed"].as<double>();
      double line_width = line_width_;
      if(data["line_width"])
        line_width = data["line_width"].as<double>();
      int branching_factor = branching_factor_;
      if(data["branching_factor"])
        branching_factor = data["branching_factor"].as<int>();
      double time_horizon = time_horizon_;
      if(data["time_horizon"])
        time_horizon = data["time_horizon"].as<double>();
      double time_minimum = time_minimum_;
      if(data["time_minimum"])
        time_minimum = data["time_minimum"].as<double>();
      double collision_checking_increment = collision_checking_increment_;
      if(data["collision_checking_increment"])
        collision_checking_increment = data["collision_checking_increment"].as<double>();
      int initial_samples = initial_samples_;
      if(data["initial_samples"])
        initial_samples = data["initial_samples"].as<int>();
      bool use_brown_paths = use_brown_paths_;
      if(data["use_brown_paths"])
        use_brown_paths = data["use_brown_paths"].as<bool>();
      int heuristic = heuristic_;
      if(data["heuristic"])
        heuristic = data["heuristic"].as<int>();
      bool gaussian_dynamic_obstacles = gaussian_dynamic_obstacles_;
      if(data["gaussian_dynamic_obstacles"])
        gaussian_dynamic_obstacles = data["gaussian_dynamic_obstacles"].as<bool>();
      bool ignore_dynamic_obstacles = ignore_dynamic_obstacles_;
      if(data["ignore_dynamic_obstacles"])
        ignore_dynamic_obstacles = data["ignore_dynamic_obstacles"].as<bool>();
      std::string planner = planner_;
      if(data["planner"])
        planner = data["planner"].as<std::string>();

      Executive::WhichPlanner which_planner = Executive::AStar;
      if (planner == "AStarPlanner")
        which_planner = Executive::AStar;
      else if (planner == "PotentialField")
        which_planner = Executive::PotentialField;
      else if (planner == "BitStar")
        which_planner = Executive::BitStar;

      executive_->setConfiguration(turning_radius, coverage_turning_radius,
                                   max_speed, slow_speed, line_width, branching_factor,
                                   heuristic, time_horizon, time_minimum,
                                   collision_checking_increment, initial_samples,
                                   use_brown_paths, gaussian_dynamic_obstacles,
                                   ignore_dynamic_obstacles, which_planner);

      executive_->startPlanner();
    }

  }

  bool running() override
  {
    if(input_task_)
      return true;
    return false;
  }

  bool getResult(std::shared_ptr<project11_navigation::Task>& output) override
  {
    iterate();
    if(output_task_)
    {
      output = output_task_;
      return true;
    }
    return false;
  }

  void iterate()
  {
    auto odom = context_->getOdometry();
    if(odom.header.stamp.isValid())
    {
      if(!trajectory_displayer_)
      {
        ros::NodeHandle nh;
        trajectory_displayer_ = std::make_shared<TrajectoryDisplayerHelper>(nh, &display_pub_, *transformations_, odom.header.frame_id);
      }
      executive_->updateCovered(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            project11::speedOverGround(odom.twist.twist.linear),
            project11::quaternionToHeadingDegrees(odom.pose.pose.orientation),
            odom.header.stamp.toSec());
    }
    // todo send contacts to executive

  }

  State publishPlan(const DubinsPlan& plan, double planning_time_ideal) override
  {
    State ret;
    auto odom = context_->getOdometry();
    ros::Duration lookahead(1.0);
    ret.setX(odom.pose.pose.position.x+odom.twist.twist.linear.x*lookahead.toSec());
    ret.setY(odom.pose.pose.position.y+odom.twist.twist.linear.y*lookahead.toSec());
    ret.setTime((odom.header.stamp+lookahead).toSec());
    ret.setYaw(tf2::getYaw(odom.pose.pose.orientation));
    ret.setSpeed(project11::speedOverGround(odom.twist.twist.linear));
    if(input_task_)
    {
      const project11_nav_msgs::Task& msg = input_task_->message();
      if(msg.poses.empty())
        return ret;

      auto data = input_task_->data();
      double step_size = step_size_;
      if(data["dubins_sample_interval"])
        step_size = data["dubins_sample_interval"].as<double>();

      project11_nav_msgs::CurvedTrajectory curved_trajectory;
      curved_trajectory.start = msg.poses.front();
      curved_trajectory.goal = msg.poses.back();
      std::vector<geometry_msgs::PoseStamped> sampled_trajectory;

      for (const auto& d : plan.get())
      {
        auto path = d.unwrap();

        double total_distance = path.param[0] + path.param[1] + path.param[2];
        ros::Time current_start_time = ros::Time(d.getStartTime());
        double speed = d.getSpeed();
        ros::Time current_end_time = ros::Time(d.getEndTime());

        if(curved_trajectory.curves.empty())
        {
          curved_trajectory.start.header.stamp = current_start_time;
          curved_trajectory.start.pose.position.x = path.qi[0];
          curved_trajectory.start.pose.position.y = path.qi[1];
          tf2::Quaternion q(tf2::Vector3(0,0,1), path.qi[2]);
          curved_trajectory.start.pose.orientation = tf2::toMsg(q);
        }
        curved_trajectory.goal.header.stamp = current_end_time;
        curved_trajectory.goal.pose.position.x = path.qi[3];
        curved_trajectory.goal.pose.position.y = path.qi[4];
        tf2::Quaternion q(tf2::Vector3(0,0,1), path.qi[5]);
        curved_trajectory.goal.pose.orientation = tf2::toMsg(q);

        project11_nav_msgs::Curve c1;
        c1.length = path.param[0];
        c1.arrival_time = current_start_time + ros::Duration(c1.length/speed);
        c1.radius = d.getRho();
        switch(path.type)
        {
          case LSL:
          case LSR:
          case LRL:
            c1.direction = c1.DIRECTION_LEFT;
            break;
          default:
            c1.direction = c1.DIRECTION_RIGHT;
        }
        curved_trajectory.curves.push_back(c1);

        project11_nav_msgs::Curve c2;
        c2.length = path.param[1];
        c2.arrival_time = c1.arrival_time + ros::Duration(c2.length/speed);
        c2.radius = d.getRho();
        switch(path.type)
        {
          case RLR:
            c2.direction = c2.DIRECTION_LEFT;
            break;
          case LRL:
            c2.direction = c2.DIRECTION_RIGHT;
            break;
          default:
            c2.direction = c2.DIRECTION_STRAIGHT;
        }
        curved_trajectory.curves.push_back(c2);

        project11_nav_msgs::Curve c3;
        c3.length = path.param[2];
        c3.arrival_time = c2.arrival_time + ros::Duration(c3.length/speed);
        c3.radius = d.getRho();
        switch(path.type)
        {
          case LSL:
          case LRL:
          case RSL:
            c3.direction = c3.DIRECTION_LEFT;
            break;
          default:
            c3.direction = c3.DIRECTION_RIGHT;
        }
        curved_trajectory.curves.push_back(c3);

        if(step_size > 0.0)
        {
          SampleContext c;
          c.pose_vector = &sampled_trajectory;
          c.start_time = current_start_time;
          c.speed = speed;
          c.frame_id = curved_trajectory.start.header.frame_id;

          int dubins_ret = dubins_path_sample_many(&path, step_size, buildPath, &c);
          if(dubins_ret != 0)
          {
            ROS_ERROR_STREAM_THROTTLE(2.0, "Error sampling Dubin's path");
          }
        }
      }

      for(auto t: input_task_->children().tasks())
        if(t->message().type == output_task_type_ && t->message().id == input_task_->getChildID(output_task_name_))
        {
          output_task_ = t;
          break;
        }
      if(!output_task_)
      {
        output_task_ = input_task_->createChildTaskBefore(std::shared_ptr<project11_navigation::Task>(),output_task_type_);
        input_task_->setChildID(output_task_, output_task_name_);
      }
      auto out_msg = output_task_->message();
      out_msg.curved_trajectories.clear();
      out_msg.curved_trajectories.push_back(curved_trajectory);
      out_msg.poses = sampled_trajectory;
      output_task_->update(out_msg);
    }
    return ret;
  }

  void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory, bool dangerous) override
  {
    if(trajectory_displayer_)
      trajectory_displayer_->displayTrajectory(trajectory, plannerTrajectory, !dangerous);
  }

  void displayDynamicObstacle(double x, double y, double yaw, double width, double length, uint32_t id) override
  {
    if(trajectory_displayer_)
    {
      geographic_visualization_msgs::GeoVizItem geoVizItem;
      std::stringstream fmt; fmt << "obstacle_" << id;
      geoVizItem.id = fmt.str();
      geographic_visualization_msgs::GeoVizPolygon polygon;
      geographic_visualization_msgs::GeoVizSimplePolygon simplePolygon;
      auto length_x = length / 2 * cos(yaw);
      auto length_y = length / 2 * sin(yaw);
      auto width_x = width / 2 * cos(yaw - M_PI_2);
      auto width_y = width / 2 * sin(yaw - M_PI_2);
      auto bow_port = trajectory_displayer_->convertToLatLong(State(x + length_x + width_x, y + length_y + width_y, 0, 0, 0));
      auto bow_starboard = trajectory_displayer_->convertToLatLong(State(x + length_x - width_x, y + length_y - width_y, 0, 0, 0));
      auto stern_port = trajectory_displayer_->convertToLatLong(State(x - length_x + width_x, y - length_y + width_y, 0, 0, 0));
      auto stern_starboard = trajectory_displayer_->convertToLatLong(State(x - length_x - width_x, y - length_y - width_y, 0, 0, 0));
      simplePolygon.points.push_back(bow_port); simplePolygon.points.push_back(bow_starboard);
      simplePolygon.points.push_back(stern_starboard); simplePolygon.points.push_back(stern_port);
      polygon.outer = simplePolygon;
      polygon.edge_color.a = 0.2;
      polygon.edge_color.b = 1;
      polygon.fill_color = polygon.edge_color;
      geoVizItem.polygons.push_back(polygon);
      display_pub_.publish(geoVizItem);
    }
  }

  void publishStats(const Planner::Stats& stats, double collisionPenalty, unsigned long cpuTime, bool lastPlanAchievable) override
  {
    alex_path_planner_common::Stats statsMsg;
    statsMsg.samples = stats.Samples;
    statsMsg.generated = stats.Generated;
    statsMsg.expanded = stats.Expanded;
    statsMsg.iterations = stats.Iterations;
    statsMsg.plan_f_value = stats.PlanFValue;
    statsMsg.plan_collision_penalty = stats.PlanCollisionPenalty;
    statsMsg.plan_time_penalty = stats.PlanTimePenalty;
    statsMsg.plan_h_value = stats.PlanHValue;
    statsMsg.plan_depth = stats.PlanDepth;
    statsMsg.collision_penalty = collisionPenalty;
    statsMsg.cpu_time = cpuTime;
    statsMsg.last_plan_achievable = lastPlanAchievable;
    stats_pub_.publish(statsMsg);
  }

  void publishTaskLevelStats(double wallClockTime, double cumulativeCollisionPenalty, double cumulativeGValue, double uncoveredLength) override
  {
    alex_path_planner_common::TaskLevelStats stats;
    stats.time = wallClockTime;
    stats.collision_penalty = cumulativeCollisionPenalty;
    stats.score = cumulativeGValue;
    stats.uncovered_length = uncoveredLength;
    task_level_stats_pub_.publish(stats);
  }

  void displayMap(std::string path) override
  {

  }

  void allDone() override
  {
    input_task_.reset();
  }

  double getTime() const override
  {
    return ros::Time::now().toSec();
  }

  void displayRibbons(const RibbonManager& ribbonManager) override
  {
    if(trajectory_displayer_)
    {
      geographic_visualization_msgs::GeoVizItem geoVizItem;
      for (const auto& r : ribbonManager.get())
      {
        geographic_visualization_msgs::GeoVizPointList displayPoints;
        displayPoints.color.r = 1;
        displayPoints.color.b = 0.5;
        displayPoints.color.a = 0.6;
        displayPoints.size = 15;
        geographic_msgs::GeoPoint point;
        displayPoints.points.push_back(trajectory_displayer_->convertToLatLong(r.startAsState()));
        displayPoints.points.push_back(trajectory_displayer_->convertToLatLong(r.endAsState()));
        geoVizItem.lines.push_back(displayPoints);
      }
      geoVizItem.id = "ribbons";
      display_pub_.publish(geoVizItem);
    }
  }

  void displayPlannerStart(const State& state)
  {
    if(trajectory_displayer_)
    {
      geographic_visualization_msgs::GeoVizItem geoVizItem;
      geographic_visualization_msgs::GeoVizPolygon polygon;
      geographic_visualization_msgs::GeoVizSimplePolygon simplePolygon;
      State bow, sternPort, sternStarboard;
      bow = state.push(3 / state.speed()); //set bow 3m ahead of state
      sternPort = state.push( -1 / state.speed());
      sternStarboard = sternPort;
      auto a = state.heading() + M_PI_2;
      auto dx = 1.5 * sin(a);
      auto dy = 1.5 * cos(a);
      sternPort.x() += dx;
      sternPort.y() += dy;
      sternStarboard.x() -= dx;
      sternStarboard.y() -= dy;
      simplePolygon.points.push_back(trajectory_displayer_->convertToLatLong(bow));
      simplePolygon.points.push_back(trajectory_displayer_->convertToLatLong(sternPort));
      simplePolygon.points.push_back(trajectory_displayer_->convertToLatLong(sternStarboard));
      polygon.outer = simplePolygon;
      polygon.edge_color.b = 1;
      polygon.edge_color.a = 0.7;
      polygon.fill_color = polygon.edge_color;
      geoVizItem.polygons.push_back(polygon);
      geoVizItem.id = "planner_start";
      display_pub_.publish(geoVizItem);
    }
  }


private:
  project11_navigation::Context::Ptr context_;
  std::shared_ptr<project11_navigation::Task> input_task_;
  std::shared_ptr<project11_navigation::Task> output_task_;

  ros::Publisher stats_pub_;
  ros::Publisher task_level_stats_pub_;
  ros::Publisher display_pub_;

  std::shared_ptr<project11::Transformations> transformations_;
  std::shared_ptr<TrajectoryDisplayerHelper> trajectory_displayer_;

  /// Segment length used for turning curves into segments
  double step_size_ = 2;

  std::string output_task_type_ = "follow_trajectory";
  std::string output_task_name_ = "navigation_trajectory";

  // handle on Executive
  std::shared_ptr<Executive> executive_;

  double turning_radius_ = 8.0;
  double coverage_turning_radius_ = 16.0;
  double max_speed_ = 2.5;
  double slow_speed_ = 0.5;
  double line_width_ = 2.0;
  int branching_factor_ = 9;
  double time_horizon_ = 30.0;
  double time_minimum_ = 5.0;
  double collision_checking_increment_= 0.05;
  int initial_samples_ = 100;
  bool use_brown_paths_ = false;
  int heuristic_ = 0;
  bool gaussian_dynamic_obstacles_ = false;
  bool ignore_dynamic_obstacles_ = false;
  std::string planner_ = "AStarPlanner";
};

} // namespace alex_path_planner

PLUGINLIB_EXPORT_CLASS(alex_path_planner::AlexPathPlanner, project11_navigation::TaskToTaskWorkflow);
