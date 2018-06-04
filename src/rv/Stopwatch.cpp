#include "rv/Stopwatch.h"
#include <cassert>

namespace rv
{

std::vector<std::chrono::system_clock::time_point> Stopwatch::stimes =
    std::vector<std::chrono::system_clock::time_point>();

void Stopwatch::tic()
{
  stimes.push_back(std::chrono::high_resolution_clock::now());
}

/** \brief stops the last timer started and outputs \a msg, if given.
 *
 *  \return elapsed time in seconds.
 **/
double Stopwatch::toc()
{
  assert(stimes.begin() != stimes.end());

  std::chrono::system_clock::time_point endtime = std::chrono::high_resolution_clock::now();
  std::chrono::system_clock::time_point starttime = stimes.back();
  stimes.pop_back();

  std::chrono::duration<double> elapsed_seconds = endtime - starttime;

  return elapsed_seconds.count();
}

}
