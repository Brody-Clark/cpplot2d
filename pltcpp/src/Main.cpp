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
#define PLTRECT_BORDEROFFSET double(0.075)

std::vector<POINT> testPoints = { {50, 50}, {100, 75}, {150, 150}, {200,100}, {800,90} };

//TODO: REMOVE
LPCWSTR ConvertToLPCWSTR(const char* charArray)
{
	int bufferSize = MultiByteToWideChar(CP_ACP, 0, charArray, -1, NULL, 0);
	wchar_t* wideString = new wchar_t[bufferSize];

	MultiByteToWideChar(CP_ACP, 0, charArray, -1, wideString, bufferSize);

	return wideString;
}

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

//TODO: Remove
std::string wcharToString(const wchar_t* wstr)
{
	// Determine the required size for the multibyte character array
	size_t length = std::wcslen(wstr) * 4 + 1;  // *4 for max bytes per wchar, +1 for null terminator
	char* str = new char[length];

	std::wcstombs(str, wstr, length);
	std::string result(str);

	delete[] str;

	return result;
}
void InitGDIPlus(Gdiplus::GdiplusStartupInput& gdiplusStartupInput, ULONG_PTR& gdiplusToken)
{
	Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
}

// Function to shutdown GDI+
void ShutdownGDIPlus(ULONG_PTR gdiplusToken)
{
	Gdiplus::GdiplusShutdown(gdiplusToken);
}

namespace pltcpp {

	double deltaX = 0, deltaY = 0; //TODO: rename (dataspanX dataspanY)
	static double mouseX = 0.0; //TODO: rename  mouseCoordX
	static double mouseY = 0.0;

	// TODO: Cleanup
	void GetLogicalDeltas(const std::vector<POINT>& points, const size_t& size, double& logicalDeltaX, double& logicalDeltaY)
	{
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

		logicalDeltaX = largestX;
		logicalDeltaY = largestY;
	}
	void DrawTextAtPosition(HDC hdc, int x, int y, const std::string& text)
	{
		SetTextColor(hdc, PLTCOLOR_WHITE);
		SetBkMode(hdc, TRANSPARENT);
		TextOut(hdc, x, y, ConvertToLPCWSTR(text.c_str()), static_cast<int>(text.length()));
	}
	void DrawPoints(HDC hdc, RECT rect, const std::vector <POINT>& points, const size_t& size)
	{
		//TODO: only need to calculate this delta once, data doesnt change.
		GetLogicalDeltas(points, size, deltaX, deltaY);

		const double xPlotOffset = rect.right * 0.075;
		const double yPlotOffset = rect.bottom * 0.075;
		const double windowDeltaX = rect.right - 2 * xPlotOffset;
		const double windowDeltaY = rect.bottom - 2 * yPlotOffset;
		double scaleX = windowDeltaX / deltaX;
		double scaleY = windowDeltaY / deltaY;

		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_GREEN);
		SelectObject(hdc, hPen);
		POINT p;
		for (int i = 0; i < size; i++)
		{
			p = points[i];
			int x = static_cast<int>((p.x * scaleX) + xPlotOffset);
			int y = static_cast<int>(windowDeltaY - (p.y * scaleY) + yPlotOffset);

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

	RECT GetPlotRect(RECT rect)
	{
		RECT out;
		const int yBorderOffset = static_cast<int>(rect.bottom * PLTRECT_BORDEROFFSET);
		const int xBorderOffset = static_cast<int>(rect.right * PLTRECT_BORDEROFFSET);

		out.left = rect.left + xBorderOffset;
		out.right = rect.right - xBorderOffset;
		out.top = rect.top + yBorderOffset;
		out.bottom = rect.bottom - yBorderOffset;

		return out;
	}

	void DrawAxes(HDC hdc, RECT rect)
	{
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_WHITE);
		SelectObject(hdc, hPen);

		RECT borderRect = GetPlotRect(rect);
		const int leftBorderPos = borderRect.left;
		const int rightBorderPos = borderRect.right;
		const int topBorderPos = borderRect.top;
		const int bottomBorderPos = borderRect.bottom;

		// Draw the border
		MoveToEx(hdc, leftBorderPos, topBorderPos, NULL);
		LineTo(hdc, rightBorderPos, topBorderPos);
		LineTo(hdc, rightBorderPos, bottomBorderPos);
		LineTo(hdc, leftBorderPos, bottomBorderPos);
		LineTo(hdc, leftBorderPos, topBorderPos);

		int tickLength = 5;

		// Draw X-axis ticks
		int numTicksX = 4;
		int x = 0;
		for (int i = 1; i <= numTicksX; ++i)
		{
			x = leftBorderPos + i * (rightBorderPos - leftBorderPos) / (numTicksX + 1);
			MoveToEx(hdc, x, bottomBorderPos + 5, NULL);
			LineTo(hdc, x, bottomBorderPos - tickLength);

			// Draw labels beneath tick marks
			std::ostringstream label;
			label << i;
			DrawTextAtPosition(hdc, x - 3, min(bottomBorderPos + 10, rect.bottom - 20), label.str());
		}

		// Draw Y-axis ticks
		int numTicksY = 4;
		int y = 0;
		for (int i = 1; i <= numTicksY; ++i)
		{
			y = topBorderPos + i * (bottomBorderPos - topBorderPos) / (numTicksY + 1);
			MoveToEx(hdc, leftBorderPos - 5, y, NULL);
			LineTo(hdc, leftBorderPos + tickLength, y);

			// Draw labels next to tick marks
			std::ostringstream label;
			label << i;
			DrawTextAtPosition(hdc, max(rect.left + 10, leftBorderPos - 20), y - 5, label.str());
		}

		DeleteObject(hPen);
	}
	void DrawPlot(HDC hdc, RECT rect, const std::vector<POINT>& points)
	{
		HBRUSH blackBrush = CreateSolidBrush(PLTCOLOR_BLACK);
		FillRect(hdc, &rect, blackBrush);
		DeleteObject(blackBrush);

		DrawAxes(hdc, rect);
		DrawPoints(hdc, rect, points, points.size());
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

	bool IsPointInRect(const int& x, const int& y, RECT rect)
	{
		return x >= rect.left && x <= rect.right && y >= rect.top && y <= rect.bottom;
	}

	LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		switch (uMsg)
		{
		case WM_PAINT: {
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hwnd, &ps);
			RECT rect;
			GetClientRect(hwnd, &rect);
			/*		HDC hdcMem = CreateCompatibleDC(hdc);
					HBITMAP hbmMem = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
					HGDIOBJ hOld = SelectObject(hdcMem, hbmMem);*/
			DrawPlot(hdc, rect, testPoints);
			DrawCoordinates(hdc, rect, mouseX, mouseY);
			//DrawToMemoryDC(hwnd, hdc);
			//BitBlt(hdc, 0, 0, rect.right, rect.bottom, hdcMem, 0, 0, SRCCOPY);

			////// Clean up
			//SelectObject(hdcMem, hOld);
			//DeleteObject(hbmMem);
			//DeleteDC(hdcMem);

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
					   /*case WM_GETMINMAXINFO: {
						   MINMAXINFO* minMaxInfo = (MINMAXINFO*)lParam;
						   minMaxInfo->ptMinTrackSize.x = 400;
						   minMaxInfo->ptMinTrackSize.y = 300;
					   }
											break;*/
		case WM_MOUSEMOVE: {
			// Get the mouse position
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			RECT textRect = PLTRECT_COORDS;
			RECT rect;
			GetClientRect(hwnd, &rect);
			RECT plotRect = GetPlotRect(rect);
			if (IsPointInRect(x, y, plotRect))
			{
				const double xPlotOffset = rect.right * 0.075;
				const double yPlotOffset = rect.bottom * 0.075;
				const double windowDeltaX = rect.right - 2 * xPlotOffset;
				const double windowDeltaY = rect.bottom - 2 * yPlotOffset;
				double scaleX = deltaX / windowDeltaX;
				double scaleY = deltaY / windowDeltaY;
				// Convert device coordinates to logical coordinates


				mouseX = (x - xPlotOffset) * scaleX;
				mouseY = (windowDeltaY - y + yPlotOffset) * scaleY;
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

} // tnypltcpp

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	InitGDIPlus(gdiplusStartupInput, gdiplusToken);

	const wchar_t CLASS_NAME[] = L"PlotWindowClass";

	WNDCLASS wc = {};
	wc.lpfnWndProc = pltcpp::WindowProc;
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
	return 0;
}