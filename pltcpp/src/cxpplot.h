// MIT License with Attribution Clause
// Copyright (c) 2025 Brody Clark
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, subject to the following conditions:
//
// 1. Attribution Requirement
//    Any use of this Software, including in original or modified form, must include
//    proper attribution to the original author(s) in the documentation, README, or
//    other relevant written materials.
//
// 2. Derivative Works
//    Any modified or derivative works of this Software must include a prominent notice
//    stating that the work is derived from this Software and must also comply with
//    the attribution requirements above.
//
// 3. Preservation of License
//    This license notice, including the attribution requirement, must be included in
//    all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND...

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
#elif defined(__linux__)
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#elif defined(__APPLE__)
#import <Cocoa/Cocoa.h>
#endif

#define PLTMENU_SAVE 1

#define PLTCOLOR_RED RGB(255, 0, 0)
#define PLTCOLOR_YELLOW RGB(255, 255, 0)
#define PLTCOLOR_GREEN RGB(0, 255, 0)
#define PLTCOLOR_WHITE RGB(255, 255, 255)
#define PLTCOLOR_BLACK RGB(0,0,0)

#define PLTWINDOW_MIN_X 200
#define PLTWINDOW_MIN_Y 100
#define PLTRECT_COORDS {0, 0, 200, 30}	// TODO: this needs to be set depending on OS
#define PLTBORDER_OFFSET_FACTOR float(0.105)

namespace cxpplot {

	class Plot2D {
	public:
		template<typename T>
		static void Plot(const std::vector<T>& x, const std::vector<T>& y, const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y");

	protected:

		class PlotImpl
		{
		public:
			PlotImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			virtual void InitWindow() = 0;
			virtual bool BrowseForFolder(std::string& outFolder) = 0;
			/*virtual void SavePlotToFile(const std::string& filename) = 0;
			virtual void DrawPoints(const std::vector<std::pair<float, float>>& data, const float& plotOffsetX, const float& plotOffsetY) = 0;*/

		protected:
			static std::string GetTimestamp();

			std::vector<std::pair<float, float>> data;
			std::pair<float, float> mouseDisplayCoordinates;
			std::string xLabel;
			std::string yLabel;
			std::string title;
			float dataSpanX = 0;
			float dataSpanY = 0;
			float plotDataOffsetX = 0;
			float plotDataOffsetY = 0;
			size_t datasetSize = 0;
		};

#ifdef _WIN32
		class Win32PlotImpl : public PlotImpl
		{
		public:
			Win32PlotImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			void InitWindow() override;

			virtual bool BrowseForFolder(std::string& outFolder) override;
			/*virtual void SavePlotToFile(const std::string& filename) override;
			virtual void DrawPoints(const std::vector<std::pair<float, float>>& data, const float& plotOffsetX, const float& plotOffsetY) override;*/

			void DrawPlot(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const std::pair<float, float>& plotBorderOffsets);
			void DrawPoints(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const size_t& size, const std::pair<float, float>& plotBorderOffsets);
			LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

			std::string GetFileName(const std::string& directory);


		protected:
			void InitGDIPlus(Gdiplus::GdiplusStartupInput& gdiplusStartupInput, ULONG_PTR& gdiplusToken);
			void ShutdownGDIPlus(ULONG_PTR gdiplusToken);
			HBITMAP CaptureWindowContent(HWND hwnd);
			HWND CreatePlotWindow(HINSTANCE hInstance, const std::string& title);
			void DrawTextAtPosition(HDC hdc, int x, int y, const std::string& text);
			void DrawVerticalTextAtPosition(HDC hdc, int x, int y, const std::string& text);
			void DrawVerticalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength);
			void DrawHorizontalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength);
			void DrawAxes(HDC hdc, RECT rect, const std::pair<float, float>& plotBorderOffsets);
			void DrawCoordinates(HDC hdc, RECT rect, const std::pair<float, float>& coordinates);
			void SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename);
			std::string wcharToString(const wchar_t* wcharStr);
			std::pair<float, float> GetMouseCoordinatesInDataSpace(RECT rect, const int x, const int y, const std::pair<float, float>& plotBorderOffsets);

			static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
		};
#endif


		static std::unique_ptr<PlotImpl> PlotImpl_;

	};

	std::unique_ptr<Plot2D::PlotImpl> Plot2D::PlotImpl_ = nullptr;

	Plot2D::PlotImpl::PlotImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel) :
		data(data), title(title), xLabel(xLabel), yLabel(yLabel)
	{
		datasetSize = data.size();

		std::pair<float, float> p;
		float largestX = -(std::numeric_limits<float>::max)(), largestY = -(std::numeric_limits<float>::max)();
		float smallestX = (std::numeric_limits<float>::max)(), smallestY = (std::numeric_limits<float>::max)();
		for (int i = 0; i < datasetSize; i++)
		{
			p = data[i];
			if (p.first > largestX)
			{
				largestX = p.first;
			}
			if (p.first < smallestX)
			{
				smallestX = p.first;
			}
			if (p.second > largestY)
			{
				largestY = p.second;
			}
			if (p.second < smallestY)
			{
				smallestY = p.second;
			}
		}

		dataSpanX = largestX - smallestX;
		dataSpanY = largestY - smallestY;

		// The conversion from data-space to plot-space requires both coord systems to start from (0, 0).
		// These values will be used to adjust the data points during conversion to achieve this.
		plotDataOffsetX = smallestX;
		plotDataOffsetY = smallestY;
	}
	inline std::string Plot2D::PlotImpl::GetTimestamp()
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

#pragma region Win32
#ifdef _WIN32
	inline Plot2D::Win32PlotImpl::Win32PlotImpl(const std::vector<std::pair<float, float>>& data,
		const std::string& title, const std::string& xLabel, const std::string& yLabel) : PlotImpl(data, title, xLabel, yLabel)
	{
	}
	void Plot2D::Win32PlotImpl::InitWindow()
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
	void Plot2D::Win32PlotImpl::InitGDIPlus(Gdiplus::GdiplusStartupInput& gdiplusStartupInput, ULONG_PTR& gdiplusToken)
	{
		Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
	}

	void Plot2D::Win32PlotImpl::ShutdownGDIPlus(ULONG_PTR gdiplusToken)
	{
		Gdiplus::GdiplusShutdown(gdiplusToken);
	}

	void Plot2D::Win32PlotImpl::DrawTextAtPosition(HDC hdc, int x, int y, const std::string& text)
	{
		SetTextColor(hdc, PLTCOLOR_WHITE);
		SetBkMode(hdc, TRANSPARENT);
		TextOut(hdc, x, y, std::wstring(text.begin(), text.end()).c_str(), static_cast<int>(text.length()));
	}

	void  Plot2D::Win32PlotImpl::DrawVerticalTextAtPosition(HDC hdc, int x, int y, const std::string& text)
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

	void  Plot2D::Win32PlotImpl::DrawPoints(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const size_t& size, const std::pair<float, float>& plotBorderOffsets)
	{
		const float xPlotSpan = rect.right - 2 * plotBorderOffsets.first;
		const float yPlotSpan = rect.bottom - 2 * plotBorderOffsets.second;
		const float xScale = xPlotSpan / (dataSpanX);
		const float yScale = yPlotSpan / dataSpanY;

		std::pair<float, float> point;
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_GREEN);
		SelectObject(hdc, hPen);
		for (int i = 0; i < size; i++)
		{
			point = data[i];
			int x = static_cast<int>(((point.first - plotDataOffsetX) * xScale) + plotBorderOffsets.first);
			int y = static_cast<int>(yPlotSpan - ((point.second - plotDataOffsetY) * yScale) + plotBorderOffsets.second);

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

	void  Plot2D::Win32PlotImpl::DrawVerticalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength)
	{
		MoveToEx(hdc, tickCenterX, tickCenterY - tickLength, NULL);
		LineTo(hdc, tickCenterX, tickCenterY + tickLength);
		std::stringstream label;
		label << std::setprecision(4) << tickValue;
		DrawTextAtPosition(hdc, tickCenterX - 10, min(tickCenterY + 10, rect.bottom - 20), label.str());
	}

	void  Plot2D::Win32PlotImpl::DrawHorizontalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength)
	{
		MoveToEx(hdc, tickCenterX - tickLength, tickCenterY, NULL);
		LineTo(hdc, tickCenterX + tickLength, tickCenterY);
		std::stringstream label;
		label << std::setprecision(4) << tickValue;
		DrawTextAtPosition(hdc, max(rect.left + 10, tickCenterX - 30), tickCenterY - 5, label.str());
	}

	void  Plot2D::Win32PlotImpl::DrawAxes(HDC hdc, RECT rect, const std::pair<float, float>& plotBorderOffsets)
	{
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_WHITE);
		SelectObject(hdc, hPen);

		const int leftBorderPos = static_cast<int>(rect.left + plotBorderOffsets.first);
		const int rightBorderPos = static_cast<int>(rect.right - plotBorderOffsets.first);
		const int topBorderPos = static_cast<int>(rect.top + plotBorderOffsets.second);
		const int bottomBorderPos = static_cast<int>(rect.bottom - plotBorderOffsets.second);

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
		float offset = plotDataOffsetX;
		float increment = dataSpanX / (numTicksX + 1);
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

	void  Plot2D::Win32PlotImpl::DrawCoordinates(HDC hdc, RECT rect, const std::pair<float, float>& coordinates)
	{
		// Convert coordinates to string
		std::string coordText = "X: " + std::to_string(coordinates.first) + " Y: " + std::to_string(coordinates.second);
		std::wstring coordTextW(coordText.begin(), coordText.end());

		SetTextColor(hdc, PLTCOLOR_WHITE);

		// Draw the coordinates in the top left corner
		TextOut(hdc, rect.left, rect.top + 1, coordTextW.c_str(), static_cast<int>(coordTextW.length()));
	}

	void  Plot2D::Win32PlotImpl::SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename)
	{
		Gdiplus::Bitmap bitmap(hBitmap, NULL);
		CLSID clsid;
		CLSIDFromString(L"{557CF406-1A04-11D3-9A73-0000F81EF32E}", &clsid); // PNG CLSID
		std::wstring wstr(filename.begin(), filename.end());
		bitmap.Save(wstr.c_str(), &clsid, NULL);
	}

	inline std::string  Plot2D::Win32PlotImpl::wcharToString(const wchar_t* wcharStr)
	{
		size_t len = std::wcslen(wcharStr);
		char* mbstr = new char[len * 4 + 1]; // Allocate enough space for multibyte characters
		std::wcstombs(mbstr, wcharStr, len * 4 + 1);
		std::string str(mbstr);
		delete[] mbstr;
		return str;
	}
	bool Plot2D::Win32PlotImpl::BrowseForFolder(std::string& outFolder)
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

	HBITMAP Plot2D::Win32PlotImpl::CaptureWindowContent(HWND hwnd)
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

	HWND Plot2D::Win32PlotImpl::CreatePlotWindow(HINSTANCE hInstance, const std::string& title)
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
		AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, L"File");

		SetMenu(hwnd, hMenu);
		return hwnd;
	}

	void Plot2D::Win32PlotImpl::DrawPlot(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const std::pair<float, float>& plotBorderOffsets)
	{
		HBRUSH blackBrush = CreateSolidBrush(PLTCOLOR_BLACK);
		FillRect(hdc, &rect, blackBrush);
		DeleteObject(blackBrush);
		DrawAxes(hdc, rect, plotBorderOffsets);
		DrawPoints(hdc, rect, data, data.size(), plotBorderOffsets);
	}

	inline LRESULT Plot2D::Win32PlotImpl::HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		RECT rect;
		GetClientRect(hwnd, &rect);
		const std::pair<float, float> plotBorderOffsets(rect.right * PLTBORDER_OFFSET_FACTOR,
			rect.bottom * PLTBORDER_OFFSET_FACTOR);
		switch (uMsg)
		{
		case WM_PAINT: {
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hwnd, &ps);

			DrawPlot(hdc, rect, data, plotBorderOffsets);
			DrawCoordinates(hdc, rect, mouseDisplayCoordinates);
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
					std::string name = GetFileName(dir);
					HBITMAP hBitmap = CaptureWindowContent(hwnd);
					SaveHBITMAPToFile(hBitmap, name);
					DeleteObject(hBitmap);
					std::wstringstream wss;
					wss << L"Image Saved to" << std::wstring(name.begin(), name.end()) << L".";
					MessageBox(hwnd, wss.str().c_str(), L"Saved", MB_OK);
				}
			}
			break;
		}
		case WM_GETMINMAXINFO: {
			MINMAXINFO* minMaxInfo = (MINMAXINFO*)lParam;
			minMaxInfo->ptMinTrackSize.x = PLTWINDOW_MIN_X;
			minMaxInfo->ptMinTrackSize.y = PLTWINDOW_MIN_Y;
		}
							 break;
		case WM_MOUSEMOVE: {
			// Get the mouse position
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			RECT textRect = PLTRECT_COORDS;
			mouseDisplayCoordinates = GetMouseCoordinatesInDataSpace(rect, x, y, plotBorderOffsets);
			InvalidateRect(hwnd, &textRect, TRUE);
			break;

		}
		case WM_SETCURSOR: {
			if (LOWORD(lParam) == HTCLIENT)
			{
				SetCursor(LoadCursor(NULL, IDC_ARROW));
				return TRUE;
			}
			else
			{
				return DefWindowProc(hwnd, uMsg, wParam, lParam);
			}
		}
		case WM_DESTROY:
			PostQuitMessage(0);
			return 0;
		}
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
	}
	std::string Plot2D::Win32PlotImpl::GetFileName(const std::string& directory)
	{
		std::string name = directory + "\\" + title + "_" + GetTimestamp() + ".png";
		std::replace(name.begin(), name.end(), ':', '_');
		return name;
	}
	std::pair<float, float> Plot2D::Win32PlotImpl::GetMouseCoordinatesInDataSpace(RECT rect, const int x, const int y, const std::pair<float, float>& plotBorderOffset)
	{
		if ((x >= plotBorderOffset.first && x <= rect.right - plotBorderOffset.first) &&
			(y >= plotBorderOffset.second && y <= rect.bottom - plotBorderOffset.second))
		{
			// Scale absolute mouse coordinates to plot window space
			const float xPlotSpan = rect.right - 2 * plotBorderOffset.first;
			const float yPlotSpan = rect.bottom - 2 * plotBorderOffset.second;
			float xScale = dataSpanX / xPlotSpan;
			float yScale = dataSpanY / yPlotSpan;

			// In win32 windows, (0,0) is upper left corner, so Y needs to be reversed
			return std::pair<float, float>((x - plotBorderOffset.first) * xScale + plotDataOffsetX,
				((yPlotSpan - y + plotBorderOffset.second) * yScale + plotDataOffsetY));
		}
		else
		{
			// Outside the plot window just default to (0,0)
			return std::pair<float, float>(0.0f, 0.0f);
		}
	}
	LRESULT CALLBACK Plot2D::Win32PlotImpl::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		Win32PlotImpl* pThis = nullptr;
		if (uMsg == WM_CREATE)
		{
			CREATESTRUCT* pCreate = reinterpret_cast<CREATESTRUCT*>(lParam);
			pThis = reinterpret_cast<Win32PlotImpl*>(pCreate->lpCreateParams);
			SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
		}
		else
		{
			pThis = reinterpret_cast<Win32PlotImpl*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
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
#pragma endregion

	template<typename T>
	void Plot2D::Plot(const std::vector<T>& x, const std::vector<T>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel)
	{
		assert(y.size() == x.size());

		std::vector<std::pair<float, float>> points;
		for (size_t i = 0; i < x.size(); ++i)
		{
			points.emplace_back(static_cast<float>(x[i]), static_cast<float>(y[i]));
		}

#ifdef _WIN32
		PlotImpl_ = std::make_unique<Win32PlotImpl>(points, title, xLabel, yLabel);
#elif defined(__APPLE__)
		PlotImpl_ = std::make_unique<CocoaPlotImpl>(points, title, xLabel, yLabel);
#elif defined(__linux__)		
		PlotImpl_ = std::make_unique<X11PlotImpl>(points, title, xLabel, yLabel);
#else
		std::throw(std::exception("Unknown platform."));
#endif

		PlotImpl_->InitWindow();

	}

	// Explicit template instantiation for numeric types
	template void Plot2D::Plot<int>(const std::vector<int>& x, const std::vector<int>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<float>(const std::vector<float>& x, const std::vector<float>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<double>(const std::vector<double>& x, const std::vector<double>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<long>(const std::vector<long>& x, const std::vector<long>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<short>(const std::vector<short>& x, const std::vector<short>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);

} // cxpplot