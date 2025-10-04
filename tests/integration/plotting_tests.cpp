#include <windows>
#include "../../include/cpplot2d.h"

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int CmdShow)
{
    std::vector<float> x;
    std::vector<float> y;

    cpplot2d::Plot2D plot(x, y);
    plot.Show();

    cpplot2d::Plot2D plot(x, y, "Distance vs Time", "distance", "time");
    plot.Show();
}
#elif defined(__linux__)

#elif defined(__APPLE__)

#endif
