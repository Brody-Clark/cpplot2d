#include <benchmark/benchmark.h>
#include "cpplot2d.h"


class Plot2DBenchmark : public cpplot2d::Plot2D
{
    public:
        Plot2DBenchmark(const std::vector<float>& x, const std::vector<float>& y,
                    const std::string& title = "Plot")
        : cpplot2d::Plot2D(x, y, title)
        {
        }
        Plot2D::GuiPolyline TestGetDataPolyline(const std::vector<std::pair<float, float>>& data,
            cpplot2d::Color color, IWindow& window)
        {
            return GetDataPolyline(data, color, window);
        }

    // Minimal mock window implementing only what's required by GetDataPolyline
    class MockWindow : public cpplot2d::Plot2D::IWindow
    {
       public:
        MockWindow(int width, int height) : m_width(width), m_height(height)
        {
        }
        void Invalidate(const WindowState& windowState) override
        {
        }
        int GetAverageCharWidth() override
        {
            return 8;
        }
        void DrawWindowState() override
        {
        }
        void InvalidateRegion(const WindowRect& rect, const WindowState& windowState) override
        {
        }
        void AddMenuButton(const std::string menu, const std::string label,
                           std::function<void()> onClickCallback) override
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

        // Inherited via IWindow
        void DrawTextAt(const std::string text, int spacing, int size, Point startPos,
                        Orientation orientation, cpplot2d::Color color) override
        {
        }
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

    // Construct Plot2D (this may initialize GDI+ / create a window depending on platform)
    Plot2DBenchmark plot(xs, ys, "benchmark_plot");

    // Build data vector to pass to GetDataPolyline (must match datasetSize inside Plot2D)
    std::vector<std::pair<float, float>> data;
    data.reserve(N);
    for (int i = 0; i < N; ++i) data.emplace_back(xs[i], ys[i]);

    Plot2DBenchmark::MockWindow mockWindow(800, 600);

    for (auto _ : state)
    {
        auto poly = plot.TestGetDataPolyline(data, cpplot2d::Color::Green(), mockWindow);
        benchmark::DoNotOptimize(poly);
    }
}

// Register benchmark with a few sizes; tune to your machine
BENCHMARK(BM_GetDataPolyline)->Arg(100)->Arg(10000)->Arg(100000)->Arg(1000000);

BENCHMARK_MAIN();