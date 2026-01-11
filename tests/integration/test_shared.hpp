#include <catch2/catch_test_macros.hpp>
#define CPPLOT2D_IMPLEMENTATION
#define CPPLOT2D_TEST
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
    
    cpplot2d::PlotProperties props;
    props.theme = cpplot2d::Theme::HighContrast();
    props.showLegend = true;
    props.showGridLines = false;
    cpplot2d::Plot2D plot("small", "Test X", "Test Y", props);

    cpplot2d::ScatterStyle style1;
    style1.radius = 3;
    cpplot2d::ScatterProperties sProps;
    sProps.style = style1;
    sProps.label = "WWWWWWWWWWWWWWWWWWWWWWWWW";
    plot.AddPoints(x, y, sProps).AddLine(x, y);
    plot.Show();
}
TEST_CASE("Text Overflow", "[integration][performance][UI][text_overflow]")
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x2;
    std::vector<float> y2;

    GetStraightDataset(x, y, 2);
    GetStraightDataset(x2, y2, 2);

    cpplot2d::PlotProperties props;
    props.theme = cpplot2d::Theme::HighContrast();
    props.showLegend = true;
    props.showGridLines = false;
    std::string title(513, 'W');
    std::string xLabel(513, 'X');
    std::string yLabel(513, 'Y');
    cpplot2d::Plot2D plot(title, xLabel, yLabel, props);

    cpplot2d::ScatterStyle style1;
    style1.radius = 3;
    cpplot2d::ScatterProperties sProps;
    sProps.style = style1;
    sProps.label = "WWWWWWWWWWWWWWWWWWWWWWWWW";
    plot.AddPoints(x, y, sProps).AddLine(x, y);
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
    cpplot2d::ScatterStyle style1;
    style1.radius = 3;
    cpplot2d::ScatterProperties sProps;
    sProps.style = style1;
    sProps.label = "custom label";
    plot.AddPoints(x, y)
        .AddPoints(x2, y2, sProps);
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
    plot.AddPoints(x, y)
        .AddPoints(x2, y2);
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
    plot.AddLine(x, y)
        .AddLine(x2, y2);
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
    plot.AddLine(x, y)
        .AddLine(x2, y2);
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
    plot.AddLine(x, y)
        .AddLine(x2, y2);
    plot.Show();
}