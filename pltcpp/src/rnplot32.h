#pragma once
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring> 
#include <ctime>
#include <cwchar>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#include <commdlg.h>
#include <gdiplus.h> // This must be inlcuded after windows.h
#pragma comment(lib, "gdiplus.lib")
#endif

#define PLTMENU_SAVE 1
#define PLTMENU_EXPORT 2

#define PLTCOLOR_RED RGB(255, 0, 0)
#define PLTCOLOR_YELLOW RGB(255, 255, 0)
#define PLTCOLOR_GREEN RGB(0, 255, 0)
#define PLTCOLOR_WHITE RGB(255, 255, 255)
#define PLTCOLOR_BLACK RGB(0,0,0)

#define PLTRECT_COORDS {0, 0, 200, 30}
#define PLTBORDER_OFFSET double(0.105)

namespace RnPlot32 {

	class Plotter2D {
	public:
		template<typename T>
		static void Plot(const std::vector<T>& x, const std::vector<T>& y, const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y");

	protected:

		class PlotterImpl
		{
		public:
			PlotterImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			virtual void InitWindow() = 0;
			virtual bool BrowseForFolder(std::string& outFolder) = 0;
			/*virtual void SavePlotToFile(const std::string& filename) = 0;
			virtual void DrawPoints(const std::vector<std::pair<float, float>>& data, const double& plotOffsetX, const double& plotOffsetY) = 0;*/



		protected:
			static std::string GetTimestamp();

			std::vector<std::pair<float, float>> data;
			std::string xLabel;
			std::string yLabel;
			std::string title;
			double dataSpanX;
			double dataSpanY;
			double plotDataOffsetX;
			double plotDataOffsetY;
			double mouseDisplayCoordX;
			double mouseDisplayCoordY;
			int datasetSize_;
		};

#ifdef _WIN32
		class Win32PlotterImpl : public PlotterImpl
		{
		public:
			Win32PlotterImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			void InitWindow() override;

			virtual bool BrowseForFolder(std::string& outFolder) override;
			/*virtual void SavePlotToFile(const std::string& filename) override;
			virtual void DrawPoints(const std::vector<std::pair<float, float>>& data, const double& plotOffsetX, const double& plotOffsetY) override;*/

			void DrawPlot(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const double& plotOffsetX, const double& plotOffsetY);
			void DrawPoints(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const size_t& size, const double& plotOffsetX, const double& plotOffsetY);
			LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
			//{
			//	switch (uMsg)
			//	{
			//	case WM_PAINT:
			//		// Handle painting
			//		return 0;
			//	case WM_DESTROY:
			//		PostQuitMessage(0);
			//		return 0;
			//	default:
			//		return DefWindowProc(hwnd, uMsg, wParam, lParam);
			//	}
			//}
		protected:
			void InitGDIPlus(Gdiplus::GdiplusStartupInput& gdiplusStartupInput, ULONG_PTR& gdiplusToken);
			void ShutdownGDIPlus(ULONG_PTR gdiplusToken);
			HBITMAP CaptureWindowContent(HWND hwnd);
			HWND CreatePlotWindow(HINSTANCE hInstance, const std::string& title);
			void DrawTextAtPosition(HDC hdc, int x, int y, const std::string& text);
			void DrawVerticalTextAtPosition(HDC hdc, int x, int y, const std::string& text);
			void DrawVerticalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const double tickValue, const int tickLength);
			void DrawHorizontalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const double tickValue, const int tickLength);
			void DrawAxes(HDC hdc, RECT rect, const double& plotOffsetX, const double& plotOffsetY);
			void DrawCoordinates(HDC hdc, RECT rect, double x, double y);
			void SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename);
			std::string wcharToString(const wchar_t* wcharStr);

			static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
		};
#endif


		static std::unique_ptr<PlotterImpl> plotterImpl_;

	};
	std::unique_ptr<Plotter2D::PlotterImpl> Plotter2D::plotterImpl_ = nullptr;

	Plotter2D::PlotterImpl::PlotterImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel) :
		data(data), title(title), xLabel(xLabel), yLabel(yLabel)
	{
	}
	inline std::string Plotter2D::PlotterImpl::GetTimestamp()
	{
		auto now = std::chrono::system_clock::now();
		std::time_t now_time = std::chrono::system_clock::to_time_t(now);
		std::string timestamp;

		std::tm local_time;
		localtime_s(&local_time, &now_time);
		std::ostringstream oss;
		oss << std::put_time(&local_time, "%Y%m%d%H%M%S");
		timestamp = oss.str();

		return timestamp;
	}
#pragma region win32
#ifdef _WIN32
	inline Plotter2D::Win32PlotterImpl::Win32PlotterImpl(const std::vector<std::pair<float, float>>& data,
		const std::string& title, const std::string& xLabel, const std::string& yLabel) : PlotterImpl(data, title, xLabel, yLabel)
	{
	}
	void Plotter2D::Win32PlotterImpl::InitWindow()
	{

		Gdiplus::GdiplusStartupInput gdiplusStartupInput;
		ULONG_PTR gdiplusToken;
		InitGDIPlus(gdiplusStartupInput, gdiplusToken);

		HWND hwnd = CreatePlotWindow(NULL, title);

		ShowWindow(hwnd, SW_SHOW); //TODO: need nCmdShow?
		UpdateWindow(hwnd);

		MSG msg = {};
		while (GetMessage(&msg, NULL, 0, 0))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		ShutdownGDIPlus(gdiplusToken);

	}
	void Plotter2D::Win32PlotterImpl::InitGDIPlus(Gdiplus::GdiplusStartupInput& gdiplusStartupInput, ULONG_PTR& gdiplusToken)
	{
		Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
	}

	void Plotter2D::Win32PlotterImpl::ShutdownGDIPlus(ULONG_PTR gdiplusToken)
	{
		Gdiplus::GdiplusShutdown(gdiplusToken);
	}

	void Plotter2D::Win32PlotterImpl::DrawTextAtPosition(HDC hdc, int x, int y, const std::string& text)
	{
		SetTextColor(hdc, PLTCOLOR_WHITE);
		SetBkMode(hdc, TRANSPARENT);
		TextOut(hdc, x, y, std::wstring(text.begin(), text.end()).c_str(), static_cast<int>(text.length()));
	}

	void  Plotter2D::Win32PlotterImpl::DrawVerticalTextAtPosition(HDC hdc, int x, int y, const std::string& text)
	{
		SetTextColor(hdc, PLTCOLOR_WHITE);
		SetBkMode(hdc, TRANSPARENT);
		std::wstring wtext(text.begin(), text.end());
		// Each letter should be drawn lower than previous
		for (wchar_t ch : wtext)
		{
			TextOut(hdc, x, y, &ch, 1);
			y += 12;
		}
	}


	void  Plotter2D::Win32PlotterImpl::DrawPoints(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const size_t& size, const double& plotOffsetX, const double& plotOffsetY)
	{
		const double xPlotSpan = rect.right - 2 * plotOffsetX;
		const double yPlotSpan = rect.bottom - 2 * plotOffsetY;
		const double xScale = xPlotSpan / (dataSpanX);
		const double yScale = yPlotSpan / dataSpanY;

		std::pair<float, float> point;
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_GREEN);
		SelectObject(hdc, hPen);
		for (int i = 0; i < size; i++)
		{
			point = data[i];
			int x = static_cast<int>(((point.first - plotDataOffsetX) * xScale) + plotOffsetX);
			int y = static_cast<int>(yPlotSpan - ((point.second - plotDataOffsetY) * yScale) + plotOffsetY);

			if (i > 0)
			{
				LineTo(hdc, x, y);
			}
			else
			{
				MoveToEx(hdc, x, y, NULL);
			}
		}
		DeleteObject(hPen);
	}

	void  Plotter2D::Win32PlotterImpl::DrawVerticalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const double tickValue, const int tickLength)
	{
		MoveToEx(hdc, tickCenterX, tickCenterY - tickLength, NULL);
		LineTo(hdc, tickCenterX, tickCenterY + tickLength);
		std::stringstream label;
		label << std::setprecision(4) << tickValue;
		DrawTextAtPosition(hdc, tickCenterX - 10, min(tickCenterY + 10, rect.bottom - 20), label.str());

	}

	void  Plotter2D::Win32PlotterImpl::DrawHorizontalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const double tickValue, const int tickLength)
	{
		MoveToEx(hdc, tickCenterX - tickLength, tickCenterY, NULL);
		LineTo(hdc, tickCenterX + tickLength, tickCenterY);
		std::stringstream label;
		label << std::setprecision(4) << tickValue;
		DrawTextAtPosition(hdc, max(rect.left + 10, tickCenterX - 30), tickCenterY - 5, label.str());

	}

	void  Plotter2D::Win32PlotterImpl::DrawAxes(HDC hdc, RECT rect, const double& plotOffsetX, const double& plotOffsetY)
	{
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_WHITE);
		SelectObject(hdc, hPen);

		const int leftBorderPos = static_cast<int>(rect.left + plotOffsetX);
		const int rightBorderPos = static_cast<int>(rect.right - plotOffsetX);
		const int topBorderPos = static_cast<int>(rect.top + plotOffsetY);
		const int bottomBorderPos = static_cast<int>(rect.bottom - plotOffsetY);

		// Draw the border
		MoveToEx(hdc, leftBorderPos, topBorderPos, NULL);
		LineTo(hdc, rightBorderPos, topBorderPos);
		LineTo(hdc, rightBorderPos, bottomBorderPos);
		LineTo(hdc, leftBorderPos, bottomBorderPos);
		LineTo(hdc, leftBorderPos, topBorderPos);

		// Draw labels
		DrawTextAtPosition(hdc, rect.right / 2, rect.bottom - 20, xLabel);
		DrawVerticalTextAtPosition(hdc, rect.left + 10, rect.bottom / 2, yLabel);

		int tickLength = 4;
		std::wstringstream label;

		// Draw X-axis ticks
		int numTicksX = 4;
		int x = 0;
		double offset = plotDataOffsetX;
		double increment = dataSpanX / (numTicksX + 1);
		for (int i = 1; i <= numTicksX; ++i)
		{
			x = leftBorderPos + i * (rightBorderPos - leftBorderPos) / (numTicksX + 1);
			offset += increment;
			DrawVerticalTick(hdc, rect, x, bottomBorderPos, offset, tickLength);
		}
		DrawVerticalTick(hdc, rect, rightBorderPos, bottomBorderPos, offset + increment, tickLength);

		// Draw Y-axis ticks
		int numTicksY = 4;
		int y = 0;
		increment = dataSpanY / (numTicksY + 1);
		offset = plotDataOffsetY;
		for (int i = 1; i <= numTicksY; ++i)
		{
			y = topBorderPos + i * (bottomBorderPos - topBorderPos) / (numTicksY + 1);
			offset += increment;
			DrawHorizontalTick(hdc, rect, leftBorderPos, y, offset, tickLength);
		}
		DrawHorizontalTick(hdc, rect, leftBorderPos, topBorderPos, offset + increment, tickLength);

		DeleteObject(hPen);
	}

	void  Plotter2D::Win32PlotterImpl::DrawCoordinates(HDC hdc, RECT rect, double x, double y)
	{
		// Convert coordinates to string
		std::string coordText = "X: " + std::to_string(x) + " Y: " + std::to_string(y);
		std::wstring coordTextW(coordText.begin(), coordText.end());

		SetTextColor(hdc, PLTCOLOR_WHITE);

		// Draw the coordinates in the top left corner
		TextOut(hdc, rect.left, rect.top + 1, coordTextW.c_str(), static_cast<int>(coordTextW.length()));
	}

	// Save HBITMAP as PNG
	void  Plotter2D::Win32PlotterImpl::SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename)
	{
		Gdiplus::Bitmap bitmap(hBitmap, NULL);
		CLSID clsid;
		CLSIDFromString(L"{557CF406-1A04-11D3-9A73-0000F81EF32E}", &clsid); // PNG CLSID
		std::wstring wstr(filename.begin(), filename.end());
		bitmap.Save(wstr.c_str(), &clsid, NULL);
	}

	inline std::string  Plotter2D::Win32PlotterImpl::wcharToString(const wchar_t* wcharStr)
	{
		size_t len = std::wcslen(wcharStr);
		char* mbstr = new char[len * 4 + 1]; // Allocate enough space for multibyte characters
		std::wcstombs(mbstr, wcharStr, len * 4 + 1);
		std::string str(mbstr);
		delete[] mbstr;
		return str;
	}
	bool Plotter2D::Win32PlotterImpl::BrowseForFolder(std::string& outFolder)
	{
		BROWSEINFO bi;
		ZeroMemory(&bi, sizeof(bi));
		WCHAR szDisplayName[MAX_PATH];
		WCHAR szPath[MAX_PATH];

		bi.hwndOwner = NULL;
		bi.pidlRoot = NULL;
		bi.pszDisplayName = szDisplayName;
		bi.lpszTitle = L"Choose Destination Folder";
		bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
		bi.lpfn = NULL;
		bi.lParam = 0;
		bi.iImage = 0;

		LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
		if (pidl != NULL)
		{
			if (SHGetPathFromIDList(pidl, szPath))
			{
				std::string result(wcharToString(szPath));
				CoTaskMemFree(pidl);
				outFolder = result;
				return true;
			}
			CoTaskMemFree(pidl);
		}
		return false;
	}
	HBITMAP Plotter2D::Win32PlotterImpl::CaptureWindowContent(HWND hwnd)
	{
		RECT rect;
		GetClientRect(hwnd, &rect);
		HDC hdcWindow = GetDC(hwnd);
		HDC hdcMemDC = CreateCompatibleDC(hdcWindow);

		HBITMAP hbmScreen = CreateCompatibleBitmap(hdcWindow, rect.right - rect.left, rect.bottom - rect.top);
		SelectObject(hdcMemDC, hbmScreen);

		BitBlt(hdcMemDC, 0, 0, rect.right - rect.left, rect.bottom - rect.top, hdcWindow, 0, 0, SRCCOPY);

		DeleteDC(hdcMemDC);
		ReleaseDC(hwnd, hdcWindow);

		return hbmScreen;
	}
	HWND Plotter2D::Win32PlotterImpl::CreatePlotWindow(HINSTANCE hInstance, const std::string& title)
	{
		const wchar_t CLASS_NAME[] = L"PlotWindowClass";

		WNDCLASS wc = {};
		wc.lpfnWndProc = WindowProc;
		wc.hInstance = hInstance;
		wc.lpszClassName = CLASS_NAME;
		wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

		RegisterClass(&wc);

		HWND hwnd = CreateWindowEx(
			0,
			CLASS_NAME,
			std::wstring(title.begin(), title.end()).c_str(),
			WS_OVERLAPPEDWINDOW,
			CW_USEDEFAULT,
			CW_USEDEFAULT,
			800,
			600,
			NULL,
			NULL,
			hInstance,
			this);

		if (hwnd == NULL)
		{
			return 0;
		}

		// Create the menu
		HMENU hMenu = CreateMenu();
		HMENU hFileMenu = CreateMenu();

		AppendMenu(hFileMenu, MF_STRING, PLTMENU_SAVE, L"Save");
		AppendMenu(hFileMenu, MF_STRING, PLTMENU_EXPORT, L"Export To CSV");
		AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, L"File");

		SetMenu(hwnd, hMenu);
		return hwnd;
	}

	void Plotter2D::Win32PlotterImpl::DrawPlot(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const double& plotOffsetX, const double& plotOffsetY)
	{
		HBRUSH blackBrush = CreateSolidBrush(PLTCOLOR_BLACK);
		FillRect(hdc, &rect, blackBrush);
		DeleteObject(blackBrush);
		DrawAxes(hdc, rect, plotOffsetX, plotOffsetY);
		DrawPoints(hdc, rect, data, data.size(), plotOffsetX, plotOffsetY);
	}
	inline LRESULT Plotter2D::Win32PlotterImpl::HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		RECT rect;
		GetClientRect(hwnd, &rect);
		const double plotOffsetX = rect.right * PLTBORDER_OFFSET;
		const double plotOffsetY = rect.bottom * PLTBORDER_OFFSET;
		switch (uMsg)
		{
		case WM_PAINT: {
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hwnd, &ps);

			DrawPlot(hdc, rect, data, plotOffsetX, plotOffsetY);
			DrawCoordinates(hdc, rect, mouseDisplayCoordX, mouseDisplayCoordY);
			EndPaint(hwnd, &ps);
		}
					 return 0;
		case WM_SIZE: {
			InvalidateRect(hwnd, NULL, TRUE);
		}
					return 0;
		case WM_COMMAND: {
			int wmId = LOWORD(wParam);
			if (wmId == PLTMENU_SAVE)
			{
				std::string dir;
				if (BrowseForFolder(dir))
				{
					std::string name = dir + "\\" + title + "_" + GetTimestamp() + ".png";
					std::replace(name.begin(), name.end(), ':', '_');
					HBITMAP hBitmap = CaptureWindowContent(hwnd);
					SaveHBITMAPToFile(hBitmap, name);
					DeleteObject(hBitmap);
					MessageBox(hwnd, L"Image saved.", L"Saved", MB_OK);	//TODO: tell where
				}
			}
			break;
		}
		case WM_GETMINMAXINFO: {
			MINMAXINFO* minMaxInfo = (MINMAXINFO*)lParam;
			minMaxInfo->ptMinTrackSize.x = 200;
			minMaxInfo->ptMinTrackSize.y = 100;
		}
							 break;
		case WM_MOUSEMOVE: {
			// Get the mouse position
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			RECT textRect = PLTRECT_COORDS;

			const double xPlotOffset = rect.right * PLTBORDER_OFFSET;
			const double yPlotOffset = rect.bottom * PLTBORDER_OFFSET;
			if ((x >= xPlotOffset && x <= rect.right - xPlotOffset) &&
				(y >= yPlotOffset && y <= rect.bottom - yPlotOffset))
			{
				const double xPlotSpan = rect.right - 2 * xPlotOffset;
				const double yPlotSpan = rect.bottom - 2 * yPlotOffset;
				double xScale = dataSpanX / xPlotSpan;
				double yScale = dataSpanY / yPlotSpan;
				// Convert device coordinates to logical coordinates


				mouseDisplayCoordX = (x - xPlotOffset) * xScale + plotDataOffsetX;
				mouseDisplayCoordY = (yPlotSpan - y + yPlotOffset) * yScale + plotDataOffsetY;
				// Invalidate the window to trigger a repaint
				InvalidateRect(hwnd, &textRect, TRUE);
				break;
			}
			else
			{
				mouseDisplayCoordX = 0;
				mouseDisplayCoordY = 0;
				InvalidateRect(hwnd, &textRect, FALSE);
				break;
			}

		}
		case WM_SETCURSOR: {
			if (LOWORD(lParam) == HTCLIENT)
			{
				SetCursor(LoadCursor(NULL, IDC_ARROW));
				return TRUE;  // Return TRUE to indicate that we handled the message
			}
			else
			{
				// Let DefWindowProc handle other cases, such as resizing
				return DefWindowProc(hwnd, uMsg, wParam, lParam);
			}
		}
		case WM_DESTROY:
			PostQuitMessage(0);
			return 0;
		}
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
	}
	LRESULT CALLBACK Plotter2D::Win32PlotterImpl::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		Win32PlotterImpl* pThis = nullptr;
		if (uMsg == WM_CREATE)
		{
			CREATESTRUCT* pCreate = reinterpret_cast<CREATESTRUCT*>(lParam);
			pThis = reinterpret_cast<Win32PlotterImpl*>(pCreate->lpCreateParams);
			SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
		}
		else
		{
			pThis = reinterpret_cast<Win32PlotterImpl*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
		}

		if (pThis)
		{
			return pThis->HandleMessage(hwnd, uMsg, wParam, lParam);
		}
		else
		{
			return DefWindowProc(hwnd, uMsg, wParam, lParam);
		}
	}

#endif
#pragma endregion win32

	template<typename T>
	void Plotter2D::Plot(const std::vector<T>& x, const std::vector<T>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel)
	{
		assert(y.size() == x.size());

		std::vector<std::pair<float, float>> points;
		for (size_t i = 0; i < x.size(); ++i)
		{
			points.emplace_back(static_cast<float>(x[i]), static_cast<float>(x[i]));
		}

#ifdef _WIN32
		plotterImpl_ = std::make_unique<Win32PlotterImpl>(points, title, xLabel, yLabel);
#endif

		plotterImpl_->InitWindow();
		/*datasetSize = x.size();

		xLabel = xLabel;
		yLabel = yLabel;

		Gdiplus::GdiplusStartupInput gdiplusStartupInput;
		ULONG_PTR gdiplusToken;
		InitGDIPlus(gdiplusStartupInput, gdiplusToken);

		HWND hwnd = CreatePlotWindow(NULL, title)

			ShowWindow(hwnd, nCmdShow);
		UpdateWindow(hwnd);

		MSG msg = {};
		while (GetMessage(&msg, NULL, 0, 0))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		ShutdownGDIPlus(gdiplusToken);*/

	}




	// Explicit template instantiation for numeric types

	/*template void Plotter2D::Plot<int>(const std::vector<int>& x, const std::vector<int>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plotter2D::Plot<float>(const std::vector<float>& x, const std::vector<float>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plotter2D::Plot<double>(const std::vector<double>& x, const std::vector<double>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plotter2D::Plot<long>(const std::vector<long>& x, const std::vector<long>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plotter2D::Plot<short>(const std::vector<short>& x, const std::vector<short>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);*/
	/*template void Plotter2D::DrawPlot<int>(HDC hdc, RECT rect, const std::vector<std::vector<int>>& points);
	template void Plotter2D::DrawPlot<float>(HDC hdc, RECT rect, const std::vector<std::vector<float>>& points);
	template void Plotter2D::DrawPlot<double>(HDC hdc, RECT rect, const std::vector < std::vector<double*/
}