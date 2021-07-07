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
    cerr << "dubins_path_type: detected LSL" << endl;
    return DubinsPathType::LSL;
    }
  if (path_type_str.compare("LSR") == 0) {
    cerr << "dubins_path_type: detected LSR" << endl;
    return DubinsPathType::LSR;
    }
  if (path_type_str.compare("RSL") == 0) {
    cerr << "dubins_path_type: detected RSL" << endl;
    return DubinsPathType::RSL;
    }
  if (path_type_str.compare("RSR") == 0) {
    cerr << "dubins_path_type: detected RSR" << endl;
    return DubinsPathType::RSR;
    }
  if (path_type_str.compare("RLR") == 0) {
    cerr << "dubins_path_type: detected RLR" << endl;
    return DubinsPathType::RLR;
    }
  if (path_type_str.compare("LRL") == 0) {
    cerr << "dubins_path_type: detected LRL" << endl;
    return DubinsPathType::LRL;
    }
  throw invalid_argument("Unrecognized path_type_str for dubins_path_type().");
}

Planner::Stats BitStarPlanner::plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
                         const DubinsPlan& previousPlan, double timeRemaining) {

    *config.output() << "DEBUG: BitStarPlanner::plan() starting" << endl;
    config.output()->flush();
    // copied from Planner.cpp
    m_Config = std::move(config);

    // TODO pick go pose from ribbon manager
    // per Roland: pick start point of first line in list

    // generate ASCII static obstacle map by
    
    // get bounds of world:
    // per Map.h:
    //  an array of four doubles, "(minX, maxX, minY, maxY)"
    //  default value: {-DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX}
    // GridWorldMap overwrites these values based on map file dimensions and resolution.
    *m_Config.output() << "DEBUG: BitStarPlanner::plan() about to get extremes" << endl;
    m_Config.output()->flush();
    auto mapExtremes = m_Config.map()->extremes();
    *m_Config.output() << "DEBUG: BitStarPlanner::plan() just got extremes: " << mapExtremes[0] << "," << mapExtremes[1] << "," << mapExtremes[2] << "," << mapExtremes[3] << "," << endl;
    config.output()->flush();
    auto mapResolution = m_Config.map()->resolution();
    *m_Config.output() << "DEBUG: BitStarPlanner::plan() just got resolution: " << mapResolution << endl;
    m_Config.output()->flush();
    // convert to number of rows and number of columns, so we'll know what ranges to index into config.map()->isBlocked()
    int num_cols = (mapExtremes[1] - mapExtremes[0]) / mapResolution;
    int num_rows = (mapExtremes[3] - mapExtremes[2]) / mapResolution;
    *m_Config.output() << "DEBUG: BitStarPlanner::plan() thinks the static obstacle map has " << num_cols << " columns and " << num_rows << " rows" << endl;

    // start build ascii world string for BIT* planner app to consume via stdin
    std::ostringstream world;
    world << mapResolution << endl;
    for (int row = 0; row < num_rows; row++) {
      for (int col = 0; col < num_cols; col++) {
        // QUESTION should I actually pass (double row.1, double col.1) to isBlocked to make sure I'm on the intended side of each cell boundary?
        if (m_Config.map()->isBlocked(col * mapResolution, row * mapResolution)) {
          world << "#";
        } else {
          world << "_";
        }
        // *m_Config.output() << "DEBUG: BitStarPlanner::plan() building world:" << endl << world << endl;
      }
      world << endl;
    }

    // start coordinates
    double start_heading = convert_eon_to_noe(start.heading());

    // goal coordinates
    State goal = selectGoal(start, ribbonManager);
    double goal_heading = convert_eon_to_noe(goal.heading());

    std::string world_str = world.str();

    *m_Config.output() << "DEBUG: BitStarPlanner constructed world:\n" << world_str << endl;

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
            char arg1[] = "-v";
            char arg2[] = "dubins";
            char arg3[] = "-u";
            char arg4[] = "1";
            char arg5[] = "-t";
            // hard-coded time limit
            char arg6[] = "10.0";
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

            // TODO add seed for RNG for reproducibility during development: -e 0

            *m_Config.output() << "DEBUG: BitStarPlanner CHILD will make this system call: " << arg0 << " " << arg1 << " " << arg2 << " " << arg3 << " " << arg4 << " " << arg5 << " " << arg6 << " " << arg7 << " " << arg8 << " " << arg9 << " " << arg10 << " " << arg11 << " " << arg12 << " " << arg13 << " " << arg14 << " " << arg15 << " " << arg16 << " " << arg17 << " " << arg18 << endl;

            execl(arg0, arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16, arg17, arg18, NULL);
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

        string chunk;
        stringstream raw_plan;
        // WORKS: exits loop after message received
        // *m_Config.output() << "DEBUG: BitStarPlanner about to get plan chunks: " << endl;
        while (std::getline(*reader, chunk)) {
          // STUB
          // *m_Config.output() << "DEBUG: BitStarPlanner got plan chunk: " << chunk << endl;
          raw_plan << chunk << endl;
        }
        // *m_Config.output() << "DEBUG: BitStarPlanner thinks it got all the plan chunks: " << endl;

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
        *m_Config.output() << "DEBUG: BitStarPlanner received following raw plan:\n" << endl;
        *m_Config.output() << raw_plan.str() << "------------" << endl;
        int batch_number;
        raw_plan >> batch_number;
        m_Stats.Iterations = batch_number + 1;
        float plan_cost;
        raw_plan >> plan_cost;
        m_Stats.PlanFValue = plan_cost;
        int solution_steps_count;
        raw_plan >> solution_steps_count;
        printf("solution with cost %f has %d steps found in batch %d\n", plan_cost, solution_steps_count, batch_number);
        double start_time = m_Config.startStateTime();
        for (int i = 1; i <= solution_steps_count; i++) {
          double qi[3] = {0,0,0};
          double param[3] = {0,0,0};
          double rho = 0;
          printf("step %d initialized qi[0] to %f\n", i, qi[0]);
          string dubins_word_str;
          // ignore standalone first print out of initial configuration (x, y, theta)
          string SKIP = "";
          raw_plan >> SKIP;
          raw_plan >> SKIP;
          raw_plan >> SKIP;
          SKIP = "";
          // get initial configuration from 
          raw_plan >> qi[0];
          printf("step %d updated qi[0] to %f\n", i, qi[0]);
          raw_plan >> qi[1];
          raw_plan >> qi[2];
          // get normalized segment lengths
          raw_plan >> param[0];
          raw_plan >> param[1];
          raw_plan >> param[2];
          // get radius/scaling factor
          raw_plan >> rho;
          // get Dubins word (path type, e.g., "LSL," etc.)
          raw_plan >> dubins_word_str;
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
            1,
            start_time
          );
          printf("step %d created DubinsWrapper with length %f\n", i, dubins_wrapper.length());

          // Update start_time for next DubinsWrapper by just using end time from this DubinsWrapper
          // (Note: DubinsWrapper.m_EndTime = m_StartTime + length() / m_Speed)
          start_time = dubins_wrapper.getEndTime();


          m_Stats.Plan.append(dubins_wrapper);
          printf(
            "step %d updated DubinsPlan (with a %i DubinsWrapper). DubinsPlan now has totalTime %f\n",
            i,
            dubins_wrapper.unwrap().type,
            m_Stats.Plan.totalTime()
          );
        }

        int status;
        pid_t wpid = waitpid(pid, &status, 0); // wait for child before terminating
        printf("parent exits\n");
        // deprecated (from development prior to incorporating into Project 11):
        // return wpid == pid && WIFEXITED(status) ? WEXITSTATUS(status) : -1;

    }

    // DEBUG


    return m_Stats;
}

