#pragma once
#include <limits>
#include <string>
#include <type_traits>
#include <vector>
#include <windows.h>

// Pre-defined max macro causes issue with limit max
#undef max

#define PLTMENU_PRINT 1
#define PLTMENU_EXPORT 2
#define PLTMENU_ABOUT 3

#define PLTCOLOR_RED RGB(255, 0, 0)
#define PLTCOLOR_YELLOW RGB(255, 255, 0)
#define PLTCOLOR_GREEN RGB(0, 255, 0)

namespace pltcpp {

	template<typename T>
	struct pltPoint {
	public:
		pltPoint(T x, T y) : (x, y) {}
		T x;
		T y;
	};
	class plt {
	public:
		template<typename T>
		static void Plot(const std::vector<pltPoint<T>>& points);

	protected:
		static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
		static bool Print();
		static bool ExportCSV();
		template<typename T>
		static void DrawPlot(HDC hdc, RECT rect, const std::vector<pltPoint<T>>& points);

		static std::vector<POINT> plotPoints;
	};


	template<typename T>
	void plt::Plot(const std::vector<pltPoint<T>>& points)
	{

	}

	template<typename T>
	void plt::DrawPlot(HDC hdc, RECT rect, const std::vector<pltPoint<T>>& points)
	{

	}
	bool plt::Print()
	{
		return true;
	}
	bool plt::ExportCSV()
	{
		return true;
	}
	LRESULT CALLBACK plt::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		return 0;
	}

	// Explicit template instantiation for numeric types
	template void plt::Plot<int>(const std::vector<pltPoint<int>>& points);
	template void plt::Plot<float>(const std::vector<pltPoint<float>>& points);
	template void plt::Plot<double>(const std::vector<pltPoint<double>>& points);
	/*template void plt::DrawPlot<int>(HDC hdc, RECT rect, const std::vector<pltPoint<int>>& points);
	template void plt::DrawPlot<float>(HDC hdc, RECT rect, const std::vector<pltPoint<float>>& points);
	template void plt::DrawPlot<double>(HDC hdc, RECT rect, const std::vector < pltPoint<double*/
}