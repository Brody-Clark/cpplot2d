#include <catch2/catch_test_macros.hpp>
#define CPPLOT2D_IMPLEMENTATION
#include "../../include/cpplot2d.h"

static void GetStraightDataset(std::vector<float>& x,std::vector<float>& y, int size)
{
    x.resize(size);
    y.resize(size);

    for(int i = 0; i < size; i++)
    {
        x[i] = i;
        y[i] = i;
    }

}
TEST_CASE("Small Scatter Plots", "[integration][performance][scatter][small]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 2);
    GetStraightDataset(x2, y2, 2);
    
    cpplot2d::Plot2D plot("small", "Test X", "Test Y");
    plot.AddPoints(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddPoints(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0), 3);
    plot.Show();
}
TEST_CASE("Medium Scatter Plots", "[integration][performance][scatter][medium]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 20000);
    GetStraightDataset(x2, y2, 20000);
    
    cpplot2d::Plot2D plot("medium", "Test X", "Test Y");
    plot.AddPoints(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddPoints(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0), 3);
    plot.Show();
}
TEST_CASE("Large Scatter Plots", "[integration][performance][scatter][large]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 2000000);
    GetStraightDataset(x2, y2, 2000000);
    
    cpplot2d::Plot2D plot("large", "Test X", "Test Y");
    plot.AddPoints(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddPoints(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0), 3);
    plot.Show();
}
TEST_CASE("Small Line Plots", "[integration][performance][line][small]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 2);
    GetStraightDataset(x2, y2, 2);
    
    
    cpplot2d::Plot2D plot("small", "Test X", "Test Y");
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();
}
TEST_CASE("Medium Line Plots", "[integration][performance][line][medium]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 20000);
    GetStraightDataset(x2, y2, 20000);
    

    cpplot2d::Plot2D plot("medium", "Test X", "Test Y");
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();
}

TEST_CASE("Large Line Plots", "[integration][performance][line][large]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 2000000);
    GetStraightDataset(x2, y2, 2000000);
    
    
    cpplot2d::Plot2D plot("large", "Test X", "Test Y");
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();
}