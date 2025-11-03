#include <windows.h>
#include "../../include/cpplot2d.h"

#ifdef _WIN32
int main()
{
    std::vector<float> x(200000);
    std::vector<float> y(200000);
    for (int i = 0; i < 100000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
    }
    for (int i = 100000; i < 200000; i++)
    {
        x[i] = static_cast<float>(i) * 0.1f;
        y[i] = static_cast<float>(i) * 0.1f;
    }
    cpplot2d::Plot2D plot(x, y);
    plot.Show();

    cpplot2d::Plot2D plot2(x, y, "Distance vs Time", "distance", "time");
    plot2.Show();
}
#elif defined(__linux__)

#elif defined(__APPLE__)

#endif
