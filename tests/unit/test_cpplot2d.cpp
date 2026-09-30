#include <catch2/catch_test_macros.hpp>
#include <vector>
#define CPPLOT2D_IMPLEMENTATION
#define CPPLOT2D_HEADLESS
#define CPPLOT2D_TEST
#include "../../include/cpplot2d.h"

namespace cpplot2d
{
class Plot2DTestAccessor
{
   public:
    using Point = Plot2D::Point;
    using WindowRect = Plot2D::WindowRect;

    static bool DoPointsIntersectRect(Plot2D& plot, const Point& p1, const Point& p2,
                                      const WindowRect& rect)
    {
        return plot.DoPointsIntersectRect(p1, p2, rect);
    }
};
}  // namespace cpplot2d

TEST_CASE("Basic test", "[plot]")
{
    std::vector<float> x(5000);
    std::vector<float> y(5000);
    for (int i = 0; i < 5000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
    }
    cpplot2d::Plot2D plot;
    CHECK(true);
}

TEST_CASE("Segment intersects rectangle", "[geometry]")
{
    cpplot2d::Plot2D plot;
    const cpplot2d::Plot2DTestAccessor::WindowRect rect(10, 0, 10, 0);

    CHECK(cpplot2d::Plot2DTestAccessor::DoPointsIntersectRect(
        plot, {-5, 5}, {15, 5}, rect));
    CHECK_FALSE(cpplot2d::Plot2DTestAccessor::DoPointsIntersectRect(
        plot, {-5, 12}, {15, 12}, rect));
    CHECK(cpplot2d::Plot2DTestAccessor::DoPointsIntersectRect(
        plot, {-1, 9}, {1, 11}, rect));
}
