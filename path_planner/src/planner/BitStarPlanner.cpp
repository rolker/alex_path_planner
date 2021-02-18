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
    world << num_cols << endl;
    world << num_rows << endl;
    for (int row = 0; row < num_rows; row++) {
      for (int col = 0; col < num_cols; col++) {
        // QUESTION should I actually pass (double row.1, doubl col.1) to isBlocked to make sure I'm on the intended side of each cell boundary?
        if (m_Config.map()->isBlocked(col, row)) {
          world << "#";
        } else {
          world << "_";
        }
        // *m_Config.output() << "DEBUG: BitStarPlanner::plan() building world:" << endl << world << endl;
      }
      world << endl;
    }

    *m_Config.output() << "DEBUG: BitStarPlanner constructed world:\n" << world.str() << endl;
    // STUB
    throw std::runtime_error("TO BE IMPLEMENTED");

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
            // https://stackoverflow.com/a/1519997
            char arg0[] = "./bit_star_planner/target/release/app";
            // confirm file is present (coarse check)
            ifstream planner_executable_file(arg0);
            if (!planner_executable_file.good()) {
              throw std::runtime_error("cannot find built executable");
            }
            char arg1[] = "-v";
            char arg2[] = "dubins";
            char arg3[] = "-u";
            char arg4[] = "1";
            char arg5[] = "-t";
            char arg6[] = "1.9";
            execl(arg0, arg0, arg1, arg2, arg3, arg4, arg5, arg6, NULL);
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

        string simple = "4\n3\n____\n____\n____\n1.0\n1.0\n2.0\n1.0";
        string space0 = "4\n3\n____\n__#_\n___#\n0.1\n1.54\n3.8\n1.1";
        string msg = simple;
        *writer << msg;
        writer->flush(); // ESSENTIAL

        close(downstream[1]); // ESSENTIAL

        string received;
        // WORKS: exits loop after message received
        while (std::getline(*reader, received)) {
            // STUB
            cout << "p" << received << endl;
            // TODO parse planner output
            // TODO initialize stats object with results from planner
        }

        int status;
        pid_t wpid = waitpid(pid, &status, 0); // wait for child before terminating
        printf("parent exits\n");
        // deprecated (from development prior to incorporating into Project 11):
        // return wpid == pid && WIFEXITED(status) ? WEXITSTATUS(status) : -1;

    }

    // STUB
    Planner::Stats results;
    results.Samples = 0;
    results.Generated = 0;
    results.Expanded = 0;
    results.Iterations = 0;
    results.PlanFValue = 0.0;
    results.PlanCollisionPenalty = 0.0;
    results.PlanTimePenalty = 0.0;
    results.PlanHValue = 0;
    results.PlanDepth = 0;
    results.Plan = DubinsPlan();

    return results;
}

