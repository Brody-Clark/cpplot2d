#include <benchmark/benchmark.h>
#define CPPLOT2D_IMPLEMENTATION
#include "cpplot2d.h"


class Plot2DBenchmark : public cpplot2d::Plot2D
{
    public:
        Plot2DBenchmark(const std::vector<float>& x, const std::vector<float>& y,
                    const std::string& title = "Plot")
        : cpplot2d::Plot2D()
        {
        }
        void TestDrawLinePlot(WindowState* state, const std::vector<float>& x,
                                                const std::vector<float>& y,
                                                int top, int left, int bottom, int right)
        {
            LineSeries series(x, y, cpplot2d::Color::Black(), 1);
            DrawLinePlot(state, WindowRect(top, left, bottom, right), series);
        }

    // Minimal mock window implementation
    class MockWindow : public cpplot2d::Plot2D::IWindow
    {
       public:
        MockWindow(int width, int height) : m_width(width), m_height(height)
        {
        }

        Dimension2d GetAverageCharSize() override
        {
            return {5, 5};
        }
        void InvalidateRegion(const WindowRect& rect, WindowState* windowState) override
        {
        }
        void AddMenuButtons(const std::string menu, MenuButtons menuButtons) override
        {
        }
        void Invalidate(WindowState* windowState) override
        {
        }
        bool SaveScreenshot(const std::string& fileName) override
        {
            return false;
        }
        void SetIsVisible(bool isVisible) override
        {
        }
        std::string GetTimestamp() override
        {
            return "ts";
        }
        WindowRect GetRect() override
        {
            return WindowRect(m_height, 0, m_width, 0);
        }
        void RunEventLoop() override
        {
        }
        void ProcessEvents() override 
        {

        }
        // Callbacks
        std::function<void(Point)> OnMouseHoverCallback;
        std::function<void()> OnResizeStartCallback;
        std::function<void()> OnResizeEndCallback;
        std::function<void()> OnResizeCallback;

        protected:
        void Draw(const GuiRect& rect) override{}
        void Draw(const GuiLine& line) override{}
        void Draw(const GuiCircle& circle) override{}
        void Draw(const GuiText& text) override{}
        void Draw(const GuiPolyline& polyline) override{}
        void Draw(const GuiPointCloud& pointcloud) override{}

       private:
        int m_width;
        int m_height;
    };
};

static void BM_GetDataPolyline(benchmark::State& state)
{
    const int N = static_cast<int>(state.range(0));

    // Prepare inputs and Plot2D instance outside timed section
    std::vector<float> xs(N), ys(N);
    for (int i = 0; i < N; ++i)
    {
        xs[i] = static_cast<float>(i);
        ys[i] = static_cast<float>(i % 100);
    }

    cpplot2d::detail::WindowState windowState;
    // Construct Plot
    Plot2DBenchmark plot(xs, ys, "benchmark_plot");
    Plot2DBenchmark::MockWindow mockWindow(800, 600);
    cpplot2d::detail::GuiPolyline line;
    for (auto _ : state)
    {
        plot.TestDrawLinePlot(&windowState, xs, ys, 600, 20, 20, 400);
        //benchmark::DoNotOptimize(poly);
    }
}

// Register benchmark with a few sizes
BENCHMARK(BM_GetDataPolyline)->Arg(100)->Arg(10000)->Arg(100000)->Arg(1000000);

BENCHMARK_MAIN();