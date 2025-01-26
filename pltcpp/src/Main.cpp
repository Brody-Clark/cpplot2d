#include <limits>
#include <string>
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

std::vector<POINT> testPoints = { {50, 50}, {100, 75}, {150, 150}, {200,100}, {800,90} };

LPCWSTR ConvertToLPCWSTR(const char* charArray)
{
	int bufferSize = MultiByteToWideChar(CP_ACP, 0, charArray, -1, NULL, 0);
	wchar_t* wideString = new wchar_t[bufferSize];

	MultiByteToWideChar(CP_ACP, 0, charArray, -1, wideString, bufferSize);

	return wideString;
}

namespace tnypltcpp {

	const int PLOT_BORDER = 100;
	double deltaX = 0, deltaY = 0;
	static double mouseX = 0.0;
	static double mouseY = 0.0;

	// TODO: Cleanup
	void GetLogicalDeltas(const std::vector<POINT>& points, const size_t& size, double& logicalDeltaX, double& logicalDeltaY)
	{
		POINT p;
		int maxInt = std::numeric_limits<int>::max();
		int largestX = -maxInt, largestY = -maxInt;
		int smallestX = maxInt, smallestY = maxInt;
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

	void DrawPoints(HDC hdc, RECT rect, const std::vector <POINT>& points, const size_t& size)
	{
		//double deltaX, delatY;
		GetLogicalDeltas(points, size, deltaX, deltaY);

		int windowDeltaX = (rect.right - PLOT_BORDER);
		int windowDeltaY = (rect.bottom - PLOT_BORDER);
		double scaleX = (windowDeltaX) / deltaX;
		double scaleY = (windowDeltaY) / deltaY;
		HPEN hPen = CreatePen(PS_SOLID, 1, PLTCOLOR_GREEN);
		SelectObject(hdc, hPen);
		POINT p;
		for (int i = 0; i < size; i++)
		{
			p = points[i];
			int x = static_cast<int>(p.x * scaleX);
			int y = static_cast<int>(p.y * scaleY);

			// Y is positive downward so reverse
			y = rect.bottom - y;

			/*SetPixel(hdc, x, y, COLOR_GREEN);*/
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

	void DrawPlot(HDC hdc, RECT rect, const std::vector<POINT>& points)
	{
		// Paint background black
		HBRUSH blackBrush = CreateSolidBrush(RGB(0, 0, 0));
		FillRect(hdc, &rect, blackBrush);
		DeleteObject(blackBrush);

		HPEN hPen = CreatePen(PS_SOLID, 1, RGB(255, 255, 255));
		SelectObject(hdc, hPen);

		// Draw the X and Y axes
		/*MoveToEx(hdc, rect.left, rect.bottom / 2, NULL);
		LineTo(hdc, rect.right, rect.bottom / 2);
		MoveToEx(hdc, rect.right / 2, rect.top, NULL);
		LineTo(hdc, rect.right / 2, rect.bottom);*/

		DeleteObject(hPen);

		// Draw some example points
		DrawPoints(hdc, rect, points, points.size());
	}

	void DrawCoordinates(HDC hdc, RECT rect, double x, double y)
	{
		// Convert coordinates to string
		std::string coordText = "X: " + std::to_string(x) + " Y: " + std::to_string(y);
		std::wstring coordTextW(coordText.begin(), coordText.end());

		// Set the text color to white
		SetTextColor(hdc, RGB(0, 0, 0));

		// Draw the coordinates in the bottom right corner
		TextOut(hdc, rect.right - 150, rect.bottom - 20, coordTextW.c_str(), coordTextW.length());
	}

	void DrawToMemoryDC(HWND hwnd, HDC hdc)
	{
		RECT rect;
		GetClientRect(hwnd, &rect);

		// Create a compatible memory DC
		HDC hdcMem = CreateCompatibleDC(hdc);
		HBITMAP hbmMem = CreateCompatibleBitmap(hdc, rect.right - rect.left, rect.bottom - rect.top);
		HGDIOBJ hOld = SelectObject(hdcMem, hbmMem);

		// Draw the plot and coordinates to the memory DC
		DrawPlot(hdcMem, rect, testPoints);
		DrawCoordinates(hdcMem, rect, mouseX, mouseY);

		// Copy the final image to the window DC
		BitBlt(hdc, 0, 0, rect.right - rect.left, rect.bottom - rect.top, hdcMem, 0, 0, SRCCOPY);

		// Cleanup
		SelectObject(hdcMem, hOld);
		DeleteObject(hbmMem);
		DeleteDC(hdcMem);
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
			DrawPlot(hdc, rect, testPoints);
			DrawCoordinates(hdc, rect, mouseX, mouseY);
			//DrawToMemoryDC(hwnd, hdc);
			EndPaint(hwnd, &ps);
		}
					 return 0;
		case WM_SIZE: {
			InvalidateRect(hwnd, NULL, TRUE);
		}
					return 0;
		case WM_COMMAND: {
			int wmId = LOWORD(wParam);
			if (wmId == PLTMENU_PRINT)
			{
				// Handle button click event
				MessageBox(hwnd, ConvertToLPCWSTR("Print button clicked!"), ConvertToLPCWSTR("Info"), MB_OK);
			}
			break;
		}
		case WM_MOUSEMOVE: {
			// Get the mouse position
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);

			// Convert device coordinates to logical coordinates
			RECT rect;
			GetClientRect(hwnd, &rect);
			double scaleX = deltaX / (rect.right - rect.left);
			double scaleY = deltaY / (rect.bottom - rect.top);

			mouseX = (x - rect.right / 2) * scaleX;
			mouseY = (rect.bottom / 2 - y) * scaleY;

			// Invalidate the window to trigger a repaint
			InvalidateRect(hwnd, NULL, FALSE);
			break;
		}
		case WM_DESTROY:
			PostQuitMessage(0);
			return 0;
		}
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
	}

	// TODO: Export
	class tnyPoint;
	void Plot(const std::vector<tnyPoint>& pts)
	{

	}
} // tnypltcpp

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	const char CLASS_NAME[] = "PlotWindowClass";

	WNDCLASS wc = {};
	wc.lpfnWndProc = tnypltcpp::WindowProc;
	wc.hInstance = hInstance;
	wc.lpszClassName = ConvertToLPCWSTR(CLASS_NAME);
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

	RegisterClass(&wc);

	HWND hwnd = CreateWindowEx(
		0,
		ConvertToLPCWSTR(CLASS_NAME),
		ConvertToLPCWSTR("Simple 2D Plot"),
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

	AppendMenu(hFileMenu, MF_STRING, PLTMENU_PRINT, ConvertToLPCWSTR("Print"));
	AppendMenu(hFileMenu, MF_STRING, PLTMENU_ABOUT, ConvertToLPCWSTR("About"));
	AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, ConvertToLPCWSTR("File"));

	SetMenu(hwnd, hMenu);

	ShowWindow(hwnd, nCmdShow);

	MSG msg = {};
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	return 0;
}