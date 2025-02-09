
#include "cxpplot.h"
#include <vector>

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	std::vector<double> xData = { 0, 1, 2, 3, 4, 5 };
	std::vector<double> yData = { 0, 1, 4, 9, 16, 25 };
	cxpplot::Plot2D::Plot(xData, yData);

	return 0;
}