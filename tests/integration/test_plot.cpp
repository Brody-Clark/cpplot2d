#include <windows.h>
#include "../../include/cpplot2d.h"

#ifdef _WIN32
int main()
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
        y2[i] = static_cast<float>(i) * 0.15f;
    }
    for (int i = 100000; i < 200000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
        x2[i] = static_cast<float>(i) * 0.15f * -1;
        y2[i] = static_cast<float>(i) * 0.15f * -1;
    }
    cpplot2d::Plot2D plot;
    plot.AddLine(x, y, cpplot2d::Color::FromRGB(0, 255, 0))
        .AddLine(x2, y2, cpplot2d::Color::FromRGB(255, 0, 0));
    plot.Show();

}
#elif defined(__linux__)

#elif defined(__APPLE__)

#endif
