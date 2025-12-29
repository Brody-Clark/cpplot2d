#include <catch2/catch_test_macros.hpp>
#include <vector>
#define CPPLOT2D_IMPLEMENTATION
#define CPPLOT2D_HEADLESS
#include "../../include/cpplot2d.h"

class TestPlot2D : public cpplot2d::Plot2D
{
    public:
    TestPlot2D(std::string title = "", std::string xLabel = "", std::string yLabel = "",
           cpplot2d::PlotProperties props = {}) : Plot2D(title, xLabel, yLabel, props)
    {

    }

    IWindow* GetMockWindow() {return m_window.get();}
    protected:

    class MockWindow : public cpplot2d::Plot2D::IWindow
    {
        public:
        MockWindow();
        ~MockWindow() override{}

    };
};

TEST_CASE("Basic test", "[plot]")
{
    std::vector<float> x(5000);
    std::vector<float> y(5000);
    for (int i = 0; i < 5000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
    }
    TestPlot2D plot;
    CHECK(true);
}
