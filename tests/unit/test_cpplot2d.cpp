#include <catch2/catch_test_macros.hpp>
#include <vector>
#include "../../include/cpplot2d.h"

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
