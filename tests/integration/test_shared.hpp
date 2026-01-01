#include <catch2/catch_test_macros.hpp>
#define CPPLOT2D_IMPLEMENTATION
#include "../../include/cpplot2d.h"

TEST_CASE("Small Scatter Plots", "[integration][performance][scatter][small]")
{
    std::vector<float> x(2);
    std::vector<float> y(2);
    std::vector<float> x2(2);
    std::vector<float> y2(2);

    x[0] = static_cast<float>(1);
    x[1] = static_cast<float>(3);
    y[0] = static_cast<float>(1);
    y[1] = static_cast<float>(3);
    x2[0] = static_cast<float>(2);
    x2[1] = static_cast<float>(4);
    y2[0] = static_cast<float>(2);
    y2[1] = static_cast<float>(2);
    
    cpplot2d::Plot2D plot("Plot", "Test X", "Test Y");
    plot.AddPoints(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddPoints(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0), 3);
    plot.Show();
}
TEST_CASE("Small Line Plots", "[integration][performance][line][small]")
{
    std::vector<float> x(2);
    std::vector<float> y(2);
    std::vector<float> x2(2);
    std::vector<float> y2(2);

    x[0] = static_cast<float>(1);
    x[1] = static_cast<float>(3);
    y[0] = static_cast<float>(1);
    y[1] = static_cast<float>(3);
    x2[0] = static_cast<float>(2);
    x2[1] = static_cast<float>(4);
    y2[0] = static_cast<float>(2);
    y2[1] = static_cast<float>(2);
    
    cpplot2d::Plot2D plot("Plot", "Test X", "Test Y");
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();
}
TEST_CASE("Medium Line Plots", "[integration][performance][line][medium]")
{
    std::vector<float> x(200000);
    std::vector<float> y(200000);
    std::vector<float> x2(200000);
    std::vector<float> y2(200000);
    for (int i = 0; i < 100000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f * -1;
        y[i] = static_cast<float>(i) * 0.1f * -1;
        x2[i] = static_cast<float>(i) * 0.15f;
        y2[i] = static_cast<float>(i) * 0.15f / (i + 1);
    }
    for (int i = 100000; i < 200000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
        x2[i] = static_cast<float>(i) * 0.15f * -1;
        y2[i] = static_cast<float>(i) * 0.15f * -1;
    }

    cpplot2d::Plot2D plot("Plot", "Test X", "Test Y");
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();
}

TEST_CASE("Large Line Plots", "[integration][performance][line][large]")
{
    std::vector<float> x(2000000);
    std::vector<float> y(2000000);
    std::vector<float> x2(2000000);
    std::vector<float> y2(2000000);
    for (int i = 0; i < 1000000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f * -1;
        y[i] = static_cast<float>(i) * 0.1f * -1;
        x2[i] = static_cast<float>(i) * 0.15f;
        y2[i] = static_cast<float>(i) * 0.15f / (i + 1);
    }
    for (int i = 1000000; i < 2000000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
        x2[i] = static_cast<float>(i) * 0.15f * -1;
        y2[i] = static_cast<float>(i) * 0.15f * -1;
    }
    
    cpplot2d::Plot2D plot("Plot", "Test X", "Test Y");
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();
}