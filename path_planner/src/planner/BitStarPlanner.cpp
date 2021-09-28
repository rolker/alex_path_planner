 // Wrapper for BIT* planner

#include "BitStarPlanner.h"
#include "Planner.h"

#include <unistd.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <sys/wait.h>
#include <iostream>

#include <sstream>

#include "../common/map/Map.h"

#include "../common/dynamic_obstacles/GaussianDynamicObstaclesManager.h"

#include <iomanip>

using namespace std;

BitStarPlanner::BitStarPlanner() {};

// buffer stuff
// copied from https://gist.github.com/rajatkhanduja/2012695 :
#include <ext/stdio_filebuf.h>

using __gnu_cxx::stdio_filebuf;
using std::istream;
using std::ostream;

using namespace std;

inline stdio_filebuf<char> * fileBufFromFD (int fd, std::_Ios_Openmode mode)
{
  return (new stdio_filebuf<char> (fd, mode));
}

istream * createInStreamFromFD (int fd) 
{
  stdio_filebuf<char> * fileBuf = fileBufFromFD (fd, std::ios::in);
  return (new istream (fileBuf)); 
}

ostream * createOutStreamFromFD (int fd) 
{
  stdio_filebuf<char> * fileBuf = fileBufFromFD (fd, std::ios::out);
  return  (new ostream (fileBuf));
}
// ... end copied buffer stuff.

State selectGoal(const State& reference, const RibbonManager& ribbonManager) {
  return ribbonManager.getNearestEndpointAsState(reference);
}

// Convert radians east of north (path_planner_common State) to radians north of east (bit_star_planner State)
double convert_eon_to_noe(double eon) {
  return fmod((M_PI / 2) - eon + 2 * M_PI, 2 * M_PI);
}

DubinsPathType dubins_path_type(string path_type_str) {
  if (path_type_str.compare("LSL") == 0) {
    // cerr << "dubins_path_type: detected LSL" << endl;
    return DubinsPathType::LSL;
    }
  if (path_type_str.compare("LSR") == 0) {
    // cerr << "dubins_path_type: detected LSR" << endl;
    return DubinsPathType::LSR;
    }
  if (path_type_str.compare("RSL") == 0) {
    // cerr << "dubins_path_type: detected RSL" << endl;
    return DubinsPathType::RSL;
    }
  if (path_type_str.compare("RSR") == 0) {
    // cerr << "dubins_path_type: detected RSR" << endl;
    return DubinsPathType::RSR;
    }
  if (path_type_str.compare("RLR") == 0) {
    // cerr << "dubins_path_type: detected RLR" << endl;
    return DubinsPathType::RLR;
    }
  if (path_type_str.compare("LRL") == 0) {
    // cerr << "dubins_path_type: detected LRL" << endl;
    return DubinsPathType::LRL;
    }
  throw invalid_argument("Unrecognized path_type_str for dubins_path_type(): " + path_type_str);
}

Planner::Stats BitStarPlanner::plan(
      const RibbonManager& ribbonManager,
      const State& start,
      PlannerConfig config,
      const DubinsPlan& previousPlan,
      double timeRemaining,
      std::unordered_map<uint32_t, GaussianDynamicObstaclesManager::Obstacle> dynamic_obstacles_copy
  ) {
    m_Config = std::move(config);
    double BitStarPlanner_plan_start_time = m_Config.now();
    *config.output() << m_Config.now() << ": DEBUG: BitStarPlanner::plan() starting" << endl;
    // cerr << "DEBUG: BitStarPlanner.plan(): dynamic_obstacles_copy.size(): " << dynamic_obstacles_copy.size() << endl;
    config.output()->flush();
    // copied from Planner.cpp

    // copied from AStarPlanner.cpp: AStarPlanner::plan()
    m_Config.setStartStateTime(start.time());

    // TODO pick go pose from ribbon manager
    // per Roland: pick start point of first line in list

    // generate ASCII static obstacle map by
    
    // get bounds of world:
    // per Map.h:
    //  an array of four doubles, "(minX, maxX, minY, maxY)"
    //  default value: {-DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX}
    // GridWorldMap overwrites these values based on map file dimensions and resolution.
    // *m_Config.output() << "DEBUG: BitStarPlanner::plan() about to get extremes" << endl;
    m_Config.output()->flush();
    auto mapExtremes = m_Config.map()->extremes();
    // *m_Config.output() << "DEBUG: BitStarPlanner::plan() just got extremes: " << mapExtremes[0] << "," << mapExtremes[1] << "," << mapExtremes[2] << "," << mapExtremes[3] << "," << endl;
    m_Config.output()->flush();
    auto mapResolution = m_Config.map()->resolution();
    // *m_Config.output() << "DEBUG: BitStarPlanner::plan() just got resolution: " << mapResolution << endl;
    m_Config.output()->flush();
    // convert to number of rows and number of columns, so we'll know what ranges to index into config.map()->isBlocked()
    int num_cols = (mapExtremes[1] - mapExtremes[0]) / mapResolution;
    int num_rows = (mapExtremes[3] - mapExtremes[2]) / mapResolution;
    // *m_Config.output() << "DEBUG: BitStarPlanner::plan() thinks the static obstacle map has " << num_cols << " columns and " << num_rows << " rows" << endl;

    // start build ascii world string for BIT* planner app to consume via stdin
    std::ostringstream world;
    world << fixed << showpoint << setprecision(9);
    world << mapResolution << endl;
    for (int row = 0; row < num_rows; row++) {
      for (int col = 0; col < num_cols; col++) {
        // QUESTION should I actually pass (double row.1, double col.1) to isBlocked to make sure I'm on the intended side of each cell boundary?
        if (m_Config.map()->isBlocked(col * mapResolution, (num_rows - row - 1) * mapResolution)) {
          world << "#";
        } else {
          world << "_";
        }
        // *m_Config.output() << "DEBUG: BitStarPlanner::plan() building world:" << endl << world << endl;
      }
      world << endl;
    }

    // serialize dynamic obstacle information
    world << "dynamic_obstacles " << dynamic_obstacles_copy.size() << endl;
    for (std::pair<uint32_t, GaussianDynamicObstaclesManager::Obstacle> entry : dynamic_obstacles_copy) {
      world << entry.first << " " << entry.second.X << " " << entry.second.Y << " " << entry.second.Yaw << " " << entry.second.Speed << " " << entry.second.Time << endl;
    }

    // start coordinates
    double start_heading = convert_eon_to_noe(start.heading());
    double start_time = m_Config.startStateTime();
    // cerr << "DEBUG: BitStarPlanner::plan got start_time = " << start_time << endl;
    // goal coordinates
    State goal = selectGoal(start, ribbonManager);
    double goal_heading = convert_eon_to_noe(goal.heading());

    std::string world_str = world.str();

    // *m_Config.output() << "DEBUG: BitStarPlanner constructed world:\n" << world_str << endl;

    // STUB
    // throw std::runtime_error("TO BE IMPLEMENTED");

    //  (3) exhaustively query map.isBlocked within extremes to set each cell to clear or blocked

    pid_t pid;

    // parent (writes) to child (who reads)
    int downstream[2];
    // child (writes) to parent (who reads)
    int upstream[2];

    pipe(downstream);
    pipe(upstream);

    pid = fork();

    if (pid == 0) {
        // child

        // child doesn't write to downstream
        close(downstream[1]);
        
        // child doesn't read from upstream 
        close(upstream[0]);

        // child's stdin is a copy of what's readable from the downstream-directed pipe
        dup2(downstream[0], STDIN_FILENO);

        // child's stdout is copied (written) into the upstream-directed pipe
        dup2(upstream[1], STDOUT_FILENO);
        
        // DEBUG switch
        int which = 1;
        if (which == 0) {
            // receive message from parent
            char msg[256];
            read(downstream[0], msg, 256);

            // send message to parent
            printf("%s polo", msg);
            // flush to make sure it gets there
            fflush(stdout);
        } else if (which == 1) {
            // absolute path to BIT* app executable on Steve's machine
            // (Since ROS nodes run in unique temporary directories, e.g. /tmp/rosmon-node-UDIhm6, there is no stable relative path
            // with current way that BIT* app executable is in bit_star_planner submodule of path_planner submodule of Project 11 repo.)
            char arg0[] = "/home/aifs2/sjw/code/project11/catkin_ws/src/path_planner/path_planner/src/planner/bit_star_planner/target/release/app";
            // confirm file is present (coarse check)
            ifstream planner_executable_file(arg0);
            if (!planner_executable_file.good()) {
              throw std::runtime_error("cannot find built executable");
            }
            // TODO should I close that ifstream since I don't use it?
            char arg1[] = "--vehicle";
            char arg2[] = "dubins";
            // char arg3[] = "--number-of-solutions";
            // const char* arg4 = std::to_string(number_of_solutions).c_str();
            char arg5[] = "--time-limit";
            // might be fine to do just timeRemaining here, but hopefully the following is more careful
            double remaining_planning_time = timeRemaining - (m_Config.now() - BitStarPlanner_plan_start_time);
            const char* arg6 = std::to_string(remaining_planning_time).c_str();
            char arg7[] = "--start-x";
            const char* arg8 = std::to_string(start.x()).c_str();
            char arg9[] = "--start-y";
            const char* arg10 = std::to_string(start.y()).c_str();
            char arg11[] = "--start-theta";
            const char* arg12 = std::to_string(start_heading).c_str();
            char arg13[] = "--goal-x";
            const char* arg14 = std::to_string(goal.x()).c_str();
            char arg15[] = "--goal-y";
            const char* arg16 = std::to_string(goal.y()).c_str();
            char arg17[] = "--goal-theta";
            const char* arg18 = std::to_string(goal_heading).c_str();
            char arg19[] = "--rho";
            double rho = m_Config.turningRadius();
            string rho_string = std::to_string(rho);
            const char* arg20 = rho_string.c_str();
            // cerr << "DEBUG: rho: " << rho << ", rho_string: " << rho_string << ", arg20: " << arg20 << endl;
            char arg21[] = "--start-time";
            // for some reason the string conversion and c_str conversion must be on separate lines for a double with as many significant digits as start_time (10 to the left, 9 to the right)
            string start_time_string = std::to_string(start_time);
            const char* arg22 = start_time_string.c_str();
            // TODO add seed for RNG for reproducibility during development: -e 0
            char arg23[] = "--dynamic-obstacle-cost-factor";
            const char* arg24 = std::to_string(dynamic_obstacle_cost_factor).c_str();
            char arg25[] = "--dynamic-obstacle-time-stdev-power";
            const char* arg26 = std::to_string(dynamic_obstacle_time_stdev_power).c_str();
            char arg27[] = "--dynamic-obstacle-time-stdev-factor";
            const char* arg28 = std::to_string(dynamic_obstacle_time_stdev_factor).c_str();

            // *m_Config.output() << m_Config.now() << ": DEBUG: BitStarPlanner CHILD will call execl with the following: " << arg0 << " " << arg1 << " " << arg2 << " " << arg3 << " " << arg4 << " " << arg5 << " " << arg6 << " " << arg7 << " " << arg8 << " " << arg9 << " " << arg10 << " " << arg11 << " " << arg12 << " " << arg13 << " " << arg14 << " " << arg15 << " " << arg16 << " " << arg17 << " " << arg18 << " " << arg19 << " " << arg20 << " " << arg21 << " " << arg22 << endl;

            execl(arg0, arg0, arg1, arg2,
              // consider commenting out the following line to suspend --number-of-solutions parameter now that proper --time-limit is set
              // and parsing takes last solution returned
              // arg3, arg4,
              arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16, arg17, arg18, arg19, arg20, arg21, arg22, arg23, arg24, arg25, arg26, arg27, arg28, NULL
            );
        }

    } else {
        // parent
        // (`else` itself probably extraneous)

        // parent doesn't read from downstream-directed pipe
        close(downstream[0]);
        // parent doesn't write to upstream-directed pipe
        close(upstream[1]);

        // buffer stuff
        ostream * writer = createOutStreamFromFD(downstream[1]);
        istream * reader = createInStreamFromFD(upstream[0]);

        // DEBUG
        string simple = "4\n3\n____\n____\n____\n1.0\n1.0\n2.0\n1.0";
        string space0 = "4\n3\n____\n__#_\n___#\n0.1\n1.54\n3.8\n1.1";

        // Set ASCII world to be sent to BIT* planner app
        string msg = world_str;
        *writer << msg;
        writer->flush(); // ESSENTIAL

        close(downstream[1]); // ESSENTIAL

        // cerr << m_Config.now() << ": DEBUG: BitStarPlanner::plan parent starts listening for reply from child." << endl;

        string chunk;
        stringstream raw_plan;
        // WORKS: exits loop after message received
        // *m_Config.output() << m_Config.now() << ": DEBUG: BitStarPlanner::plan about to get raw plan chunks." << endl;
        m_Config.output()->flush();
        while (std::getline(*reader, chunk)) {
          // *m_Config.output() << m_Config.now() << ": DEBUG: BitStarPlanner::plan parent received plan chunk: " << chunk << endl;
          m_Config.output()->flush();

          // QUESTION: Why do I append `endl` when inserting chunk into raw_plan here? Is it to recreate the lines that are removed during getline() extraction, above? Probably.
          raw_plan << chunk << endl;
          raw_plan.flush();
        }
        *m_Config.output() << m_Config.now() << ": DEBUG: BitStarPlanner parent thinks it got all the plan chunks." << endl;
        m_Config.output()->flush();

        *m_Config.output()<< m_Config.now()  << ": DEBUG: BitStarPlanner received following raw plan(s):\n" << endl;
        *m_Config.output() << raw_plan.str() << "------------" << endl;


        // // Initialize planner instance's m_Stats member,
        // // a field of which will store the plan we're about
        // // to parse from BIT*

        // m_Stats = Stats();

        // m_Stats.Samples = 0; // not currently reported by BIT*
        // m_Stats.Generated = 0; // not currently reported by BIT*
        // m_Stats.Expanded = 0; // not currently reported by BIT*
        // m_Stats.Iterations = 0; // batch_number + 1 (see re-assignment below)
        // m_Stats.PlanFValue = 0.0; // f = g + h = plan cost + 0 for BIT* (see re-assignment below)
        // m_Stats.PlanCollisionPenalty = 0.0;
        // m_Stats.PlanTimePenalty = 0.0;
        // m_Stats.PlanHValue = 0; // correct for BIT*, which returns complete plan
        // m_Stats.PlanDepth = 0; // "Isn't really used anymore, as it was just for the UCS planner" -- Alex
        // m_Stats.Plan = DubinsPlan();

        // // parse planner output
        // *m_Config.output()<< m_Config.now()  << ": DEBUG: BitStarPlanner received following raw plan(s):\n" << endl;
        // *m_Config.output() << raw_plan.str() << "------------" << endl;

        // // thither to extract value names (could theoretically perform validation on them, too) and or other ignored fields from BIT* output
        // chunk = "";
        // int solution_number;

        // // ignores all solutions prior to the solution whose solution_number equals the constant number_of_solutions set in the header
        // while (!raw_plan.eof()) {
        //   raw_plan >> chunk;
        //   if (chunk.compare(std::string("solution")) == 0) {
        //     raw_plan >> solution_number;
        //     cerr << m_Config.now() << ": DEBUG: BitStarPlanner::plan parent found solution_number = " << solution_number << endl;
        //     m_Config.output()->flush();
        //     if (solution_number == number_of_solutions) {
        //       cerr << m_Config.now() << ": DEBUG: found solution_number == " << number_of_solutions << "; breaking out of while-loop" << endl;
        //       break;
        //     }
        //   }
        // }
        // cerr << m_Config.now() << ": DEBUG: first line outside of while-loop that parses raw_plan" << endl;

        // parse raw_plan(s), keeping only the last solution
        while (!raw_plan.eof()) {
          // Initialize planner instance's m_Stats member,
          // a field of which will store the plan we're about
          // to parse from BIT*

          m_Stats = Stats();

          m_Stats.Samples = 0; // not currently reported by BIT*
          m_Stats.Generated = 0; // not currently reported by BIT*
          m_Stats.Expanded = 0; // not currently reported by BIT*
          m_Stats.Iterations = 0; // batch_number + 1 (see re-assignment below)
          m_Stats.PlanFValue = 0.0; // f = g + h = plan cost + 0 for BIT* (see re-assignment below)
          m_Stats.PlanCollisionPenalty = 0.0;
          m_Stats.PlanTimePenalty = 0.0;
          m_Stats.PlanHValue = 0; // correct for BIT*, which returns complete plan
          m_Stats.PlanDepth = 0; // "Isn't really used anymore, as it was just for the UCS planner" -- Alex
          m_Stats.Plan = DubinsPlan();

          // parse planner output
          chunk = "";
          int solution_number;
          raw_plan >> chunk;
          raw_plan >> solution_number;
          cerr<< m_Config.now() << ": DEBUG: solution_number = " << solution_number << endl;
          int batch_number;
          raw_plan >> chunk;
          raw_plan >> batch_number;
          cerr<< m_Config.now() << ": DEBUG: batch_number = " << batch_number << endl;
          m_Stats.Iterations = batch_number + 1;
          float plan_cost;
          raw_plan >> chunk;
          cerr<< m_Config.now() << ": DEBUG: got \"" << chunk << "\"" << endl;
          chunk = "";
          raw_plan >> plan_cost;
          cerr<< m_Config.now() << ": DEBUG: plan_cost = " << plan_cost << endl;
          float plan_duration;
          raw_plan >> chunk;
          cerr<< m_Config.now() << ": DEBUG: got \"" << chunk << "\"" << endl;
          chunk = "";
          raw_plan >> plan_duration;
          cerr<< m_Config.now() << ": DEBUG: plan_duration = " << plan_duration << endl;
          m_Stats.PlanFValue = plan_cost;
          int solution_steps_count;
          raw_plan >> chunk;
          cerr<< m_Config.now() << ": DEBUG: got \"" << chunk << "\"" << endl;
          chunk = "";
          raw_plan >> solution_steps_count;
          cerr<< m_Config.now() << ": DEBUG: solution_steps_count = " << solution_steps_count << endl;
          cerr << m_Config.now() << ": BitStarPlanner.plan(): solution " << solution_number << " from batch " << batch_number << " has cost " << plan_cost << " and duration " << plan_duration << 
          " (s) in " << solution_steps_count << " steps.\n";
          cerr.flush();
          double start_time = m_Config.startStateTime();
          std::cerr<< m_Config.now()  << ": BitStarPlanner.plan: got initial start_time of " << start_time << " from m_Config.startStateTime()." << std::endl;
          // parse solution steps
          for (int i = 1; i <= solution_steps_count; i++) {
            cerr << m_Config.now() << ": DEBUG: BitStarPlanner::plan parent parsing solution step " << i << endl;
            cerr.flush();
            double qi[3] = {0,0,0};
            double param[3] = {0,0,0};
            double rho = 0;
            // printf("step %d initialized qi[0] to %f\n", i, qi[0]);
            string dubins_word_str;
            // ignore standalone first print out of initial configuration (x, y, theta)
            raw_plan >> chunk;
            cerr << "DEBUG: got \"" << chunk << "\"" << endl;
            chunk = "";
            raw_plan >> chunk;
            cerr << "DEBUG: got \"" << chunk << "\"" << endl;
            chunk = "";
            raw_plan >> chunk;
            cerr << "DEBUG: got \"" << chunk << "\"" << endl;
            chunk = "";
            // get initial configuration from 
            raw_plan >> qi[0];
            cerr << "DEBUG: set qi[0] to " << qi[0] << endl;
            // printf("step %d updated qi[0] to %f\n", i, qi[0]);
            raw_plan >> qi[1];
            cerr << "DEBUG: set qi[1] to " << qi[1] << endl;
            raw_plan >> qi[2];
            cerr << "DEBUG: set qi[2] to " << qi[2] << endl;
            // get normalized segment lengths
            raw_plan >> param[0];
            cerr << "DEBUG: set param[0] to " << param[0] << endl;
            raw_plan >> param[1];
            cerr << "DEBUG: set param[1] to " << param[1] << endl;
            raw_plan >> param[2];
            cerr << "DEBUG: set param[2] to " << param[2] << endl;
            // get radius/scaling factor
            raw_plan >> rho;
            cerr << "DEBUG: set rho to " << rho << endl;
            // get Dubins word (path type, e.g., "LSL," etc.)
            raw_plan >> dubins_word_str;
            cerr << "DEBUG: set dubins_word_str to " << dubins_word_str << endl;
            // not currently used in Executive or BitStarPlanner wrapper
            double _g_cost;
            raw_plan >> _g_cost;
            // convert to proper DubinsPath struct type
            DubinsPath dubins_path = {
              {qi[0], qi[1], qi[2]},
              {param[0], param[1], param[2]},
              rho,
              dubins_path_type(dubins_word_str),
            };
            printf("step %d created DubinsPath with qi[0] of %f (%s: %i)\n", i, dubins_path.qi[0], dubins_word_str.c_str(), dubins_path.type);
            DubinsWrapper dubins_wrapper = DubinsWrapper();
            // TODO figure out correct speed and start time to set in fill() call:
            dubins_wrapper.fill(
              dubins_path,
              m_Config.maxSpeed(),
              start_time
            );
            // printf("step %d created DubinsWrapper with length %f\n", i, dubins_wrapper.length());

            // Update start_time for next DubinsWrapper by just using end time from this DubinsWrapper
            // std::cerr << "BitStarPlanner.plan: updating start_time from " << start_time << " to " << dubins_wrapper.getEndTime() << std::endl;
            // (Note: DubinsWrapper.m_EndTime = m_StartTime + length() / m_Speed)
            start_time = dubins_wrapper.getEndTime();


            m_Stats.Plan.append(dubins_wrapper);
            // printf(
            //   "step %d updated DubinsPlan (with a %i DubinsWrapper). DubinsPlan now has totalTime %f\n",
            //   i,
            //   dubins_wrapper.unwrap().type,
            //   m_Stats.Plan.totalTime()
            // );
          }
          chunk = "";
          raw_plan >> chunk;
          cerr << m_Config.now() << ": DEBUG: got " << chunk << endl;
          int tree_size;
          raw_plan >> tree_size;
          cerr << m_Config.now() << ": DEBUG: got tree_size " << tree_size << endl;
          int a, b, c, d, e, f, g, h, i, j, k, l;
          for (int tree_elt = 1; tree_elt <= tree_size; tree_elt++) {
            cerr << m_Config.now() << ": DEBUG: skipping tree element " << tree_elt << endl;
            raw_plan >> a >> b >> c >> d >> e >> f >> g >> h >> i >> j >> k >> l;
          }
        }
          
        // cerr << m_Config.now() << ": DEBUG: BitStarPlanner::plan parent finished parsing solution." << endl;
        cerr.flush();
        // NOTE: It appears we do not read the (rest of the) tree from the BIT* output. If we want to in the future, we can.

        int status;
        pid_t wpid = waitpid(pid, &status, 0); // wait for child before terminating
        // cerr << m_Config.now() << ": DEBUG: BitStarPlanner::plan parent learned that child has terminated. Terminating parent." << endl;
        cerr.flush();
        // deprecated (from development prior to incorporating into Project 11):
        // return wpid == pid && WIFEXITED(status) ? WEXITSTATUS(status) : -1;

    }

    // DEBUG


    return m_Stats;
}

