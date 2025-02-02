#include <limits>
#include <sstream>
#include <string>
#include <ctime>
#include <chrono>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <windows.h>
#include <gdiplus.h> // This must be inlcuded after windows.h
#pragma comment(lib, "gdiplus.lib")

#define PLTMENU_SAVE 1
#define PLTMENU_EXPORT 2

#define PLTCOLOR_RED RGB(255, 0, 0)
#define PLTCOLOR_YELLOW RGB(255, 255, 0)
#define PLTCOLOR_GREEN RGB(0, 255, 0)
#define PLTCOLOR_WHITE RGB(255, 255, 255)
#define PLTCOLOR_BLACK RGB(0,0,0)

#define PLTRECT_COORDS {0, 0, 200, 30}
#define PLTBORDER_OFFSET double(0.105)

std::vector<POINT> testPoints = { {-20, -20}, { 50, 50 }, {100, 75}, {150, 150}, {200,100}, {800,90} };



namespace c2pplot {

	double xDataSpan = 0, yDataSpan = 0;
	double xPlotDataOffset = 0, yPlotDataOffset = 0;
	static double mouseX = 0.0;
	static double mouseY = 0.0;
	int size = 0;

	struct Point {

	public:
		Point(double x, double y) : x(x), y(y) {}

		double x;
		double y;
	};
	std::wstring GetTimestamp()
	{
		auto now = std::chrono::system_clock::now();
		std::time_t now_time = std::chrono::system_clock::to_time_t(now);
		std::string timestamp;

		std::tm local_time;
		localtime_s(&local_time, &now_time);
		std::ostringstream oss;
		oss << std::put_time(&local_time, "%Y%m%d%H%M%S");
		timestamp = oss.str();

		std::wstring wstr(timestamp.begin(), timestamp.end());
		return wstr;
	}

	void InitGDIPlus(Gdiplus::GdiplusStartupInput& gdiplusStartupInput, ULONG_PTR& gdiplusToken)
	{
		Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
	}

	void ShutdownGDIPlus(ULONG_PTR gdiplusToken)
	{
		Gdiplus::GdiplusShutdown(gdiplusToken);
	}

	void DrawTextAtPosition(HDC hdc, int x, int y, const std::wstring& text)
	{
		SetTextColor(hdc, PLTCOLOR_WHITE);
		SetBkMode(hdc, TRANSPARENT);
		TextOut(hdc, x, y, text.c_str(), static_cast<int>(text.length()));
	}
	void DrawVerticalTextAtPosition(HDC hdc, int x, int y, const std::wstring& text)
	{
		SetTextColor(hdc, PLTCOLOR_WHITE);
		SetBkMode(hdc, TRANSPARENT);
		for (wchar_t ch : text)
		{
			TextOut(hdc, x, y, &ch, 1);
			y += 12;
		}
	}
	void DrawPoints(HDC hdc, RECT rect, const std::vector <POINT>& points, const size_t& size)
	{
		const double xPlotOffset = rect.right * PLTBORDER_OFFSET;
		const double yPlotOffset = rect.bottom * PLTBORDER_OFFSET;
		const double xPlotSpan = rect.right - 2 * xPlotOffset;
		const double yPlotSpan = rect.bottom - 2 * yPlotOffset;
		double xScale = xPlotSpan / (xDataSpan);
		double yScale = yPlotSpan / yDataSpan;

		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_GREEN);
		SelectObject(hdc, hPen);
		POINT p;
		for (int i = 0; i < size; i++)
		{
			p = points[i];
			int x = static_cast<int>(((p.x - xPlotDataOffset) * xScale) + xPlotOffset);
			int y = static_cast<int>(yPlotSpan - ((p.y - yPlotDataOffset) * yScale) + yPlotOffset);

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
	void DrawVertcialTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const double tickValue, const int tickLength)
	{
		MoveToEx(hdc, tickCenterX, tickCenterY - tickLength, NULL);
		LineTo(hdc, tickCenterX, tickCenterY + tickLength);
		std::wstringstream label;
		label << std::setprecision(4) << tickValue;
		DrawTextAtPosition(hdc, tickCenterX - 10, min(tickCenterY + 10, rect.bottom - 20), label.str());

	}
	void DrawHorizontalTick(HDC hdc, RECT rect, const int tickCenterX, const int tickCenterY, const double tickValue, const int tickLength)
	{
		MoveToEx(hdc, tickCenterX - tickLength, tickCenterY, NULL);
		LineTo(hdc, tickCenterX + tickLength, tickCenterY);
		std::wstringstream label;
		label << std::setprecision(4) << tickValue;
		DrawTextAtPosition(hdc, max(rect.left + 10, tickCenterX - 30), tickCenterY - 5, label.str());

	}
	void DrawAxes(HDC hdc, RECT rect, const Point& plotOffset)
	{
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_WHITE);
		SelectObject(hdc, hPen);

		const int leftBorderPos = static_cast<int>(rect.left + plotOffset.x);
		const int rightBorderPos = static_cast<int>(rect.right - plotOffset.x);
		const int topBorderPos = static_cast<int>(rect.top + plotOffset.y);
		const int bottomBorderPos = static_cast<int>(rect.bottom - plotOffset.y);

		// Draw the border
		MoveToEx(hdc, leftBorderPos, topBorderPos, NULL);
		LineTo(hdc, rightBorderPos, topBorderPos);
		LineTo(hdc, rightBorderPos, bottomBorderPos);
		LineTo(hdc, leftBorderPos, bottomBorderPos);
		LineTo(hdc, leftBorderPos, topBorderPos);

		// Draw labels
		DrawTextAtPosition(hdc, rect.right / 2, rect.bottom - 20, L"XXTTJJ");
		DrawVerticalTextAtPosition(hdc, rect.left + 10, rect.bottom / 2, L"XXTTJJ");

		int tickLength = 4;
		std::wstringstream label;
		// Draw X-axis ticks
		int numTicksX = 4;
		int x = 0;
		double offset = xPlotDataOffset;
		double increment = xDataSpan / (numTicksX + 1);
		for (int i = 1; i <= numTicksX; ++i)
		{
			x = leftBorderPos + i * (rightBorderPos - leftBorderPos) / (numTicksX + 1);
			offset += increment;
			DrawVertcialTick(hdc, rect, x, bottomBorderPos, offset, tickLength);
		}
		DrawVertcialTick(hdc, rect, rightBorderPos, bottomBorderPos, offset + increment, tickLength);

		// Draw Y-axis ticks
		int numTicksY = 4;
		int y = 0;
		increment = yDataSpan / (numTicksY + 1);
		offset = yPlotDataOffset;
		for (int i = 1; i <= numTicksY; ++i)
		{
			y = topBorderPos + i * (bottomBorderPos - topBorderPos) / (numTicksY + 1);
			offset += increment;
			DrawHorizontalTick(hdc, rect, leftBorderPos, y, offset, tickLength);
		}
		DrawHorizontalTick(hdc, rect, leftBorderPos, topBorderPos, offset + increment, tickLength);

		DeleteObject(hPen);
	}
	void DrawPlot(HDC hdc, RECT rect, Point plotOffsets)
	{
		HBRUSH blackBrush = CreateSolidBrush(PLTCOLOR_BLACK);
		FillRect(hdc, &rect, blackBrush);
		DeleteObject(blackBrush);

		DrawAxes(hdc, rect, plotOffsets);
		DrawPoints(hdc, rect, testPoints, testPoints.size());
	}

	void DrawCoordinates(HDC hdc, RECT rect, double x, double y)
	{
		// Convert coordinates to string
		std::string coordText = "X: " + std::to_string(x) + " Y: " + std::to_string(y);
		std::wstring coordTextW(coordText.begin(), coordText.end());

		SetTextColor(hdc, PLTCOLOR_WHITE);

		// Draw the coordinates in the top left corner
		TextOut(hdc, rect.left, rect.top + 1, coordTextW.c_str(), static_cast<int>(coordTextW.length()));
	}

	// Save HBITMAP as PNG
	void SaveHBITMAPToFile(HBITMAP hBitmap, const std::wstring& filename)
	{
		Gdiplus::Bitmap bitmap(hBitmap, NULL);
		CLSID clsid;
		CLSIDFromString(L"{557CF406-1A04-11D3-9A73-0000F81EF32E}", &clsid); // PNG CLSID
		bitmap.Save(filename.c_str(), &clsid, NULL);
	}

	HBITMAP CaptureWindowContent(HWND hwnd)
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
	void Plot(const std::vector<POINT>& points, const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y")
	{
		int size = points.size();

		POINT p;
		int largestX = -(std::numeric_limits<int>::max)(), largestY = -(std::numeric_limits<int>::max)();
		int smallestX = (std::numeric_limits<int>::max)(), smallestY = (std::numeric_limits<int>::max)();
		for (int i = 0; i < size; i++)
		{
			p = points[i];
			if (p.x > largestX)
			{
				largestX = p.x;
			}
			if (p.x < smallestX)
			{
				smallestX = p.x;
			}
			if (p.y > largestY)
			{
				largestY = p.y;
			}
			if (p.y < smallestY)
			{
				smallestY = p.y;
			}
		}

		xDataSpan = largestX - smallestX;
		yDataSpan = largestY - smallestY;

		// The conversion from data-space to plot-space requires both coord systems to start from (0, 0).
		// These values will be used to adjust the data points during conversion to achieve this.
		xPlotDataOffset = smallestX;
		yPlotDataOffset = smallestY;

	}
	LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		RECT rect;
		GetClientRect(hwnd, &rect);
		const double xPlotOffset = rect.right * PLTBORDER_OFFSET;
		const double yPlotOffset = rect.bottom * PLTBORDER_OFFSET;
		Point plotOffsets(rect.right * PLTBORDER_OFFSET, rect.bottom * PLTBORDER_OFFSET);
		switch (uMsg)
		{
		case WM_PAINT: {
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hwnd, &ps);

			DrawPlot(hdc, rect, plotOffsets);
			DrawCoordinates(hdc, rect, mouseX, mouseY);
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
				HBITMAP hBitmap = CaptureWindowContent(hwnd);
				std::wstringstream wss;
				wss << L"Plot" << L"_" << GetTimestamp() << L".png";
				SaveHBITMAPToFile(hBitmap, wss.str());
				DeleteObject(hBitmap);
				MessageBox(hwnd, L"Screenshot saved.", L"Saved", MB_OK);	//TODO: tell where
			}
			else if (wmId == PLTMENU_EXPORT)
			{
				MessageBox(hwnd, L"Data exported.", L"Exported To CSV", MB_OK);	//TODO: tell where
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
				double xScale = xDataSpan / xPlotSpan;
				double yScale = yDataSpan / yPlotSpan;
				// Convert device coordinates to logical coordinates


				mouseX = (x - xPlotOffset) * xScale + xPlotDataOffset;
				mouseY = (yPlotSpan - y + yPlotOffset) * yScale + yPlotDataOffset;
				// Invalidate the window to trigger a repaint
				InvalidateRect(hwnd, &textRect, TRUE);
				break;
			}
			else
			{
				mouseX = 0;
				mouseY = 0;
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

	int Init(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
	{
		Plot(testPoints);

		Gdiplus::GdiplusStartupInput gdiplusStartupInput;
		ULONG_PTR gdiplusToken;
		InitGDIPlus(gdiplusStartupInput, gdiplusToken);

		const wchar_t CLASS_NAME[] = L"PlotWindowClass";

		WNDCLASS wc = {};
		wc.lpfnWndProc = c2pplot::WindowProc;
		wc.hInstance = hInstance;
		wc.lpszClassName = CLASS_NAME;
		wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

		RegisterClass(&wc);

		HWND hwnd = CreateWindowEx(
			0,
			CLASS_NAME,
			L"Simple 2D Plot",
			WS_OVERLAPPEDWINDOW,
			CW_USEDEFAULT,
			CW_USEDEFAULT,
			800,
			600,
			NULL,
			NULL,
			hInstance,
			NULL);

		if (hwnd == NULL)
		{
			return 0;
		}

		// TODO: Header only file wouldnt have acccess to resources folder anyways
		/*HICON hIcon = (HICON)LoadImage(
			NULL,
			L".\\resources\\c2pplot.png",
			IMAGE_ICON,
			0,
			0,
			LR_LOADFROMFILE | LR_DEFAULTSIZE
		);

		if (hIcon == NULL)
		{
			return 1;
		}
		SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIcon);
		SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);*/

		// Create the menu
		HMENU hMenu = CreateMenu();
		HMENU hFileMenu = CreateMenu();

		AppendMenu(hFileMenu, MF_STRING, PLTMENU_SAVE, L"Save");
		AppendMenu(hFileMenu, MF_STRING, PLTMENU_EXPORT, L"Export To CSV");
		AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, L"File");

		SetMenu(hwnd, hMenu);

		ShowWindow(hwnd, nCmdShow);
		UpdateWindow(hwnd);

		MSG msg = {};
		while (GetMessage(&msg, NULL, 0, 0))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		ShutdownGDIPlus(gdiplusToken);
		//DestroyIcon(hIcon);
		return 0;
	}

} // c2pplot

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	return c2pplot::Init(hInstance, hPrevInstance, lpCmdLine, nCmdShow);
}