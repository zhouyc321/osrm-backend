#ifndef OSRM_HEAP_PROFILER
#define OSRM_HEAP_PROFILER

#include <gperftools/heap-profiler.h>

namespace osrm
{
namespace util
{

class /*Scoped*/HeapProfiler
{
  public:
    HeapProfiler(const char *prefix_) : prefix{prefix_}
    {
        if (!IsProfiling())
            ::HeapProfilerStart(prefix);
    }

    ~HeapProfiler()
    {
        if (IsProfiling())
        {
            Dump(prefix);
            ::HeapProfilerStop();
        }
    }

    bool IsProfiling() const { return ::IsHeapProfilerRunning() != 0; }
    void Dump(const char *why) const { ::HeapProfilerDump(why); }

  private:
    const char *prefix;
};
}
}

#endif
