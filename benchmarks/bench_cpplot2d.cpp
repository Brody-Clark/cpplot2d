#include <benchmark/benchmark.h>
#define CPPLOT2D_IMPLEMENTATION
#define CPPLOT2D_HEADLESS
#define CPPLOT2D_TEST
#include "cpplot2d.h"

namespace cpplot2d
{
    class Plot2DTestAccessor
    {
       public:
           using Point = Plot2D::Point;
           using WindowRect = Plot2D::WindowRect;

        static const Plot2D::DrawCommand& GetDrawCommand(const Plot2D& plot)
        {
            return plot.m_plotDrawCommand;
        }
        static Plot2D::IWindow* GetWindow(const Plot2D& plot)
        {
            return plot.m_window.get();
        }
        static bool GetIsDirty(const Plot2D& plot)
        {
            return plot.m_plotDirty;
        }
        static bool DoPointsIntersectRect(const Plot2D::Point& p1, const Plot2D::Point& p2,
                                          const Plot2D::WindowRect& rect, Plot2D& plot)
        {
            return plot.DoPointsIntersectRect(p1, p2, rect);
        }
    };
};

static void BM_DoPointsIntersectRect(benchmark::State& state)
{
    cpplot2d::Plot2D plot;
    for (auto _ : state)
    {
        bool hit = cpplot2d::Plot2DTestAccessor::DoPointsIntersectRect(
            cpplot2d::Plot2DTestAccessor::Point{10, 10}, {60, 10},
            cpplot2d::Plot2DTestAccessor::WindowRect(30, 25, 50, 5), plot);
        benchmark::DoNotOptimize(hit);
    }

}

BENCHMARK(BM_DoPointsIntersectRect);

BENCHMARK_MAIN();