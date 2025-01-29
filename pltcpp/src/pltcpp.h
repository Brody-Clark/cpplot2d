#pragma once
#include <algorithm>
#include <chrono>
#include <commdlg.h>
#include <cstdlib>
#include <cstring> 
#include <ctime>
#include <cwchar>
#include <fstream>
#include <iomanip>
#include <limits>
#include <shlobj.h>
#include <sstream>
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

	class plt {
	public:
		template<typename T>
		static void Plot(const std::vector<std::vector<T>>& points, const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y");

	protected:
		static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
		static bool Print();
		static bool ExportCSV();
		inline static void replaceChars(std::string& str, char oldChar, char newChar);

		template<typename T>
		static void DrawPlot(HDC hdc, RECT rect, const std::vector<std::vector<T>>& points);

		template <typename T>
		static void WriteLine(std::ofstream& outFile, std::vector<T> data);

		static std::string axisNameX;
		static std::string axisNameY;
		static std::vector<POINT> plotPoints;
	};

	std::string plt::axisNameX = "x";
	std::string plt::axisNameY = "y";

	inline void plt::replaceChars(std::string& str, char oldChar, char newChar)
	{
		for (char& ch : str)
		{
			if (ch == oldChar)
			{
				ch = newChar;
			}
		}
	}

	LPCWSTR ConvertToLPCWSTR(const char* charArray)
	{
		int bufferSize = MultiByteToWideChar(CP_ACP, 0, charArray, -1, NULL, 0);
		wchar_t* wideString = new wchar_t[bufferSize];

		MultiByteToWideChar(CP_ACP, 0, charArray, -1, wideString, bufferSize);

		return wideString;
	}
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
	template<typename T>
	void plt::Plot(const std::vector<std::vector<T>>& points, const std::string& title, const std::string& xLabel, const std::string& yLabel)
	{
		axisNameX = xLabel;
		axisNameY = yLabel;
		replaceChars(axisNameX, ',', '_');
		replaceChars(axisNameY, ',', '_');


	}

	template<typename T>
	void plt::DrawPlot(HDC hdc, RECT rect, const std::vector<std::vector<T>>& points)
	{

	}
	bool plt::Print()
	{
		return true;
	}
	std::string BrowseForFolder(HWND hwndOwner, const std::string& title)
	{
		BROWSEINFO bi;
		ZeroMemory(&bi, sizeof(bi));
		WCHAR szDisplayName[MAX_PATH];
		WCHAR szPath[MAX_PATH];

		bi.hwndOwner = hwndOwner;
		bi.pidlRoot = NULL;
		bi.pszDisplayName = szDisplayName;
		bi.lpszTitle = ConvertToLPCWSTR(title.c_str());
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
				return result;
			}
			CoTaskMemFree(pidl);
		}
		return std::string();
	}
	template <typename T>
	void plt::WriteLine(std::ofstream& outFile, std::vector<T> data)
	{
		size_t len = data.size();
		for (int i = 0; i < len; i++)
		{
			outFile << data[i] << (i < len - 1) ? "," : "\n";
		}
	}
	bool plt::ExportCSV()
	{
		//std::string dir = BrowseForFolder(NULL, "Select a folder to save to:");
		//std::string name = "solution_" + GetTimestamp();
		//std::replace(name.begin(), name.end(), ':', '_');
		//std::string path = dir + "\\" + name + ".csv";
		//std::ofstream outFile(path);

		//// Check if the file is open.
		//if (outFile)
		//{
		//	// Write the headers.
		//	outFile << axisNameX << "," << axisNameY << ", \n";

		//	// Write data points as csv
		//	std::vector<float> point;
		//	for (size_t i = 0; i < point.size(); i++)
		//	{
		//		point = points[i];
		//		WriteLine(outFile, point);
		//	}

		//	outFile.close();
		//}
		//else
		//{
		//	throw std::exception("Unable to open file for writing");
		//}

	}
	LRESULT CALLBACK plt::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		return 0;
	}

	// Explicit template instantiation for numeric types
	template void plt::Plot<int>(const std::vector<std::vector<int>>& points, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void plt::Plot<float>(const std::vector<std::vector<float>>& points, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void plt::Plot<double>(const std::vector<std::vector<double>>& points, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	/*template void plt::DrawPlot<int>(HDC hdc, RECT rect, const std::vector<std::vector<int>>& points);
	template void plt::DrawPlot<float>(HDC hdc, RECT rect, const std::vector<std::vector<float>>& points);
	template void plt::DrawPlot<double>(HDC hdc, RECT rect, const std::vector < std::vector<double*/
}