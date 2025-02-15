
#include "cxpplot.h"
#include <vector>

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	std::vector< long> xData;
	std::vector< long> yData;
	for (int i = 0; i < 1000; i++)
	{
		xData.push_back(i * 1000);
		yData.push_back(i * 1000);
	}
	cxpplot::Plot2D::Plot(xData, yData);

	return 0;
}