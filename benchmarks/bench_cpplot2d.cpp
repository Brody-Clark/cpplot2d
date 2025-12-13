#include <benchmark/benchmark.h>
#include "cpplot2d.h"


class Plot2DBenchmark : public cpplot2d::Plot2D
{
    public:
        Plot2DBenchmark(const std::vector<float>& x, const std::vector<float>& y,
                    const std::string& title = "Plot")
        : cpplot2d::Plot2D()
        {
        }
        Plot2D::GuiPolyline TestGetDataPolyline(const std::vector<float>& x,
                                                const std::vector<float>& y,
                                                cpplot2d::Color color, 
                                                const WindowRect& rect)
        {
            LineSeries series(x, y, cpplot2d::Color::Black(), 1);
           
            return GetDataPolyline(series, rect);
        }

    // Minimal mock window implementation
    class MockWindow : public cpplot2d::Plot2D::IWindow
    {
       public:
        MockWindow(int width, int height) : m_width(width), m_height(height)
        {
        }

        int GetAverageCharWidth() override
        {
            return 8;
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
        bool SaveScreenshotAsPNG(const std::string& fileName) override
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
        // Callbacks
        std::function<void(Point)> OnMouseHoverCallback;
        std::function<void()> OnResizeStartCallback;
        std::function<void()> OnResizeEndCallback;
        std::function<void()> OnResizeCallback;

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

    // Construct Plot
    Plot2DBenchmark plot(xs, ys, "benchmark_plot");
    Plot2DBenchmark::MockWindow mockWindow(800, 600);

    for (auto _ : state)
    {
        auto poly = plot.TestGetDataPolyline(xs, ys, cpplot2d::Color::Green(), mockWindow.GetRect());
        benchmark::DoNotOptimize(poly);
    }
}

// Register benchmark with a few sizes
BENCHMARK(BM_GetDataPolyline)->Arg(100)->Arg(10000)->Arg(100000)->Arg(1000000);

BENCHMARK_MAIN();