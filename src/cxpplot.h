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
#include <execution>
#include <thread>
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
#ifdef __OBJC__
#import <Foundation/Foundation.h>
#import <Cocoa/Cocoa.h>

#pragma mark - ObjC
// Need to define ObjC classes in global namespace
@interface PlotView : NSView
@property (nonatomic, copy) NSArray<NSValue *> *dataPoints;
@property (nonatomic) NSPoint mouseCoordinates;
@property (nonatomic, strong) NSMutableArray<NSDictionary *> *textEntries;
@property (nonatomic, strong) NSMutableArray<NSValue *> *lines;
@property (nonatomic, strong) NSTrackingArea *trackingArea;
@property (nonatomic, assign) BOOL onlyDrawCoordinates;
@end
extern NSString *const MouseMovedNotification = @"MouseMovedNotification";
@implementation PlotView
- (instancetype)initWithFrame:(NSRect)frame {
	self = [super initWithFrame:frame];
	if (self) {
		[self.window setAcceptsMouseMovedEvents:YES];
		[self addTrackingArea:[[NSTrackingArea alloc] initWithRect:self.bounds
														   options:(NSTrackingMouseMoved | NSTrackingActiveInKeyWindow)
															 owner:self
														  userInfo:nil]];
		_lines = [[NSMutableArray alloc] init];
		_onlyDrawCoordinates = false;
	}
	return self;
}

- (void)updateTrackingAreas {
	[super updateTrackingAreas];

	// Remove any existing tracking areas
	if (self.trackingArea) {
		[self removeTrackingArea:self.trackingArea];
	}

	// Create a new tracking area
	NSTrackingArea *trackingArea = [[NSTrackingArea alloc] initWithRect:self.bounds
														options:(NSTrackingMouseEnteredAndExited |
																 NSTrackingMouseMoved |
																 NSTrackingActiveInKeyWindow)
														  owner:self
													   userInfo:nil];
	[self addTrackingArea:trackingArea];
	self.trackingArea = trackingArea;
}
- (void)mouseMoved:(NSEvent *)event {
	NSPoint location = [self convertPoint:event.locationInWindow fromView:nil];

	// Post notification with mouse position
	NSDictionary *userInfo = @{@"x": @(location.x), @"y": @(location.y)};
	[[NSNotificationCenter defaultCenter] postNotificationName:MouseMovedNotification object:self userInfo:userInfo];
	
	// TODO: Handle resize cursor
//	// Get the window size
//	NSRect frame = self.bounds;
//	
//	// Thickness of the border for resizing
//	CGFloat borderThickness = 5.0;
//
//	// Check if the mouse is near any edge
//	if (fabs(location.x - frame.origin.x) < borderThickness ||  // Left edge
//		fabs(location.x - NSMaxX(frame)) < borderThickness) {  // Right edge
//		[[NSCursor resizeLeftRightCursor] set];
//	} else if (fabs(location.y - frame.origin.y) < borderThickness ||  // Bottom edge
//			   fabs(location.y - NSMaxY(frame)) < borderThickness) {  // Top edge
//		[[NSCursor resizeUpDownCursor] set];
//	} else {
//		[[NSCursor arrowCursor] set];
//	}
	
}
- (void)resetCursorRects {
	[self addCursorRect:[self bounds] cursor:[NSCursor resizeLeftRightCursor]];
}
- (void)DrawMouseCoordinates {
	if (!CGPointEqualToPoint(self.mouseCoordinates, NSZeroPoint))
	{
		// Format the coordinates as a string
		NSString *coordinateText = [NSString stringWithFormat:@"X=%.2f  Y=%.2f", self.mouseCoordinates.x, self.mouseCoordinates.y];
		
		// Set attributes for the text
		NSDictionary *mouseCoordAttr = @{
			NSFontAttributeName: [NSFont systemFontOfSize:12],
			NSForegroundColorAttributeName: [NSColor whiteColor]
		};
		
		// Draw the formatted string at a top left of the window
		NSPoint drawPoint = NSMakePoint(10, self.bounds.size.height - 20);
		[coordinateText drawAtPoint:drawPoint withAttributes:mouseCoordAttr];
	}
}

- (void)DrawPlot {
	// Set color for the axes
	[[NSColor whiteColor] setStroke];
	
	// Init current graphics context
	CGContextRef context = [[NSGraphicsContext currentContext] CGContext];
	CGContextSetStrokeColorWithColor(context, [[NSColor blackColor] CGColor]);
	CGContextSetLineWidth(context, 1.0);
	
	// Draw axis labels
	NSDictionary *axisLabelAttr = @{
		NSFontAttributeName: [NSFont systemFontOfSize:10],
		NSForegroundColorAttributeName: [NSColor whiteColor]
	};
	
	for (NSDictionary *entry in self.textEntries) {
		NSString *text = entry[@"text"];
		NSPoint position = [entry[@"position"] pointValue];
		
		[text drawAtPoint:position withAttributes:axisLabelAttr];
	}
	
	// Draw axis and tick lines
	for (NSUInteger i = 0; i < self.lines.count; i += 2) {
		
		NSPoint start, end;
		[self.lines[i] getValue:&start size:sizeof(NSPoint)];
		[self.lines[i+1] getValue:&end size:sizeof(NSPoint)];
		
		CGContextMoveToPoint(context, start.x, start.y);
		CGContextAddLineToPoint(context, end.x, end.y);
		
	}
	// Draw all strokes at the same time. Need to call this for each color
	CGContextStrokePath(context);
	
	// Draw lines to each point
	[[NSColor greenColor] setStroke]; // Set color for the plot line
	NSPoint point = [[self.dataPoints objectAtIndex:0] pointValue];
	CGContextMoveToPoint(context, point.x, point.y);
	for (NSValue *pointValue in self.dataPoints)
	{
		NSPoint point = [pointValue pointValue];
		CGContextAddLineToPoint(context, point.x, point.y);
		CGContextMoveToPoint(context, point.x, point.y);
	}
	
	// Draw all plot strokes at the same time
	CGContextStrokePath(context);
}

- (void)drawRect:(NSRect)dirtyRect
{
	[super drawRect:dirtyRect];
	
	[[NSColor blackColor] setFill];
	NSRectFill(dirtyRect);  // Clear background
	
	// Draw mouse coordinates
	[self DrawMouseCoordinates];
	
	// Quick and dirty
	// Skip redrawing anything else this pass if the user is just moving their mouse
	if(self.onlyDrawCoordinates)
	{
		return;
	}
	
	[self DrawPlot];
}

- (void)addLineFrom:(NSPoint)start to:(NSPoint)end {
	NSValue *line = [NSValue valueWithBytes:&start objCType:@encode(NSPoint)];
	NSValue *endLine = [NSValue valueWithBytes:&end objCType:@encode(NSPoint)];
	
	[self.lines addObject:line];
	[self.lines addObject:endLine];
}
- (void)draw {
	[self setNeedsDisplay:YES]; // Trigger a full redraw
}
- (void)updateData:(NSArray<NSValue *> *)newData
{
	self.dataPoints = newData;
}
- (void)updateDisplayCoordinates:(NSPoint)newCoords
{
	self.mouseCoordinates = newCoords;
	
	NSRect dirtyRect = NSMakeRect(0, self.bounds.size.height - 20,
									  200, 30);
	// Flag used in draw call to only render mouse coordinates.
	// No need to redraw all lines if the user is just moving their mouse over the plot
	self.onlyDrawCoordinates = true;
	
	// Mark only the small rect for redraw
	[self setNeedsDisplayInRect:dirtyRect];
}
- (void)addText:(NSString *)text atPosition:(NSPoint)position {
	if (!self.textEntries) {
		self.textEntries = [NSMutableArray array];
	}

	NSDictionary *textEntry = @{
		@"text": text,
		@"position": [NSValue valueWithPoint:position]
	};

	[self.textEntries addObject:textEntry];
	
}
- (void)viewDidChangeBackingProperties
{
	[super viewDidChangeBackingProperties];
	[self setNeedsDisplay:YES];
}
- (void)setFrameSize:(NSSize)newSize
{
	[super setFrameSize:newSize];
	self.onlyDrawCoordinates = false; // Resizing, so we need to redraw whole plot
	[self setNeedsDisplay:YES];
}
- (void)savePlot:(id)sender
{
	// Show a save dialog
	NSSavePanel* savePanel = [NSSavePanel savePanel];
	NSArray* fileTypes = [NSArray arrayWithObjects:@"png", nil];
	savePanel.allowedContentTypes = fileTypes;
	//[savePanel setAllowedFileTypes:@[@"png"]];
	[savePanel setNameFieldStringValue:@"plot.png"];
	
	if ([savePanel runModal] == NSModalResponseOK) {
		NSURL *saveURL = [savePanel URL];

		// Render view into an image
		NSBitmapImageRep *imageRep = [self bitmapImageRepForCachingDisplayInRect:self.bounds];
		[self cacheDisplayInRect:self.bounds toBitmapImageRep:imageRep];
		
		// Convert to PNG data
		NSData *pngData = [imageRep representationUsingType:NSBitmapImageFileTypePNG properties:@{}];

		// Write to file
		[pngData writeToURL:saveURL atomically:YES];
	}
}
@end

// Window delegate to handle close event
@interface WindowDelegate : NSObject <NSWindowDelegate>
@end

@implementation WindowDelegate

- (void)windowWillClose:(NSNotification *)notification {
	[NSApp stop:nil]; // Stops the event loop when the window closes
}

@end
#pragma mark
#pragma mark - End of ObjC
#endif
#endif

#define PLTMENU_SAVE 1

#define PLTCOLOR_RED RGB(255, 0, 0)
#define PLTCOLOR_YELLOW RGB(255, 255, 0)
#define PLTCOLOR_GREEN RGB(0, 255, 0)
#define PLTCOLOR_WHITE RGB(255, 255, 255)
#define PLTCOLOR_BLACK RGB(0,0,0)
#define PLTRECT_MOUSE_DISPLAY_COORDS {0, 0, 200, 30}    // TODO: this needs to be set depending on OS
#define PLTBORDER_OFFSET_FACTOR float(0.105)
#define CXP_MULTITHREADING_ENABLED 0
#define CXP_DEFERRED_DRAW_LIMIT 5000

namespace cxpplot {
	enum EPlotColor
	{
		WHITE = 0,
		BLACK = 1,
		BLUE = 2,
		GREEN = 3,
		YELLOW = 4,
		
		DEFAULT = 999
	};

	class Plot2D {
	public:
		template<typename T>
		Plot2D(const std::vector<T>& x, const std::vector<T>& y, const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y");
		Plot2D(const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y");
		
		/**
		 Show the plot with the pre-determined plot points and parameters.
		 */
		void Show();
		
		/**
		 Sets whether the legend should be shown on the plot or not.
		 
		 @param show whether or not to show the legend
		 */
		void DisplayLegend(bool show);
		
		/**
		 Sets whether or not to enable multithreading for data processing.
		 
		 @param enable enables multithreading if applicable
		 */
		void SetMultithreadingEnabled(bool enable);
		
		/**
		 Sets the bottom limit for deferring window redraws until the window is idle. Decrease if resizing the window causes flickering.
		 
		 @param limit the number of plot points beyond which the window will deferr redrawing until it is in an idle state.
		 */
		void SetDeferredDrawLimit(int limit = 5000);
		
		template<typename T>
		void Plot(const std::vector<T>& x, const std::vector<T>& y, const std::string& label, EPlotColor color);
		template<typename T>
		static void Plot(const std::vector<T>& x, const std::vector<T>& y, const std::string& title = "Plot", const std::string& xLabel = "x", const std::string& yLabel = "y");
	protected:
		
		class PlotImpl
		{
		public:
			PlotImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			virtual void Show() = 0;
			virtual ~PlotImpl() = default;
		protected:
			static std::string GetTimestamp();
			virtual bool BrowseForFolder(std::string& outFolder) = 0;
			virtual void OnMouseMove(const int x, const int y) = 0;
			virtual void OnResize(const int& newWidth, const int& newHeight) = 0;
			std::vector<std::pair<float, float>> data;
			std::pair<float, float> mouseDisplayCoordinates;
			std::pair<float, float> PlotZeroOffset;
			std::string xLabel;
			std::string yLabel;
			std::string title;
			float dataSpanX = 0;
			float dataSpanY = 0;
			size_t datasetSize = 0;
			const int tickLength = 4;
			const std::pair<int, int> minWindowSize = {200,100};
			std::pair<int, int> defaultWindowSize = {800,600};
			int deferredResizeLimit = 5000;
			struct Rect2D
			{
				
			public:
				Rect2D(const int top, const int bottom, const int left, const int right):
				top(top), bottom(bottom), left(left), right(right){}
			
				Rect2D(const float top, const float bottom, const float left, const float right):
				top(static_cast<int>(top)),
				bottom(static_cast<int>(bottom)),
				left(static_cast<int>(left)),
				right(static_cast<int>(right)){}
				
				int top;
				int bottom;
				int left;
				int right;
			};
			
			struct Range2D
			{
				float smallestX;
				float smallestY;
				float largestX;
				float largestY;
			};
			enum EDrawMask
			{
				DRAW_ALL = 0,
				DRAW_MOUSE_COORIDNATES = 1,
				DRAW_PLOT = 2
			};
			struct PlotProps
			{
				int deferredDrawLimit = 5000;
				bool multithreadingEnabled = false;
				int innerPlotPaddingPercent = 2;
				int ylabelOffsetPercent = 2;
				int xlabelOffsetPercent = 2;
				
			};
			
			PlotProps props;
			EDrawMask drawMask = EDrawMask::DRAW_ALL;
			
		};

#ifdef _WIN32
		class Win32PlotImpl : public PlotImpl
		{
		public:
			Win32PlotImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			~Win32PlotImpl() override;
			void Show() override;

			virtual bool BrowseForFolder(std::string& outFolder) override;

			void DrawPlot(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const std::pair<float, float>& plotBorderOffsets);
			void DrawPoints(HDC hdc, RECT rect, const std::vector<std::pair<float, float>>& data, const size_t& size, const std::pair<float, float>& plotBorderOffsets);
			LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

			std::string GetFileName(const std::string& directory);

		protected:
			bool isResizing;
			bool isMouseMoving;
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
#ifdef __APPLE__
#ifdef __OBJC__
	  class CocoaPlotImpl : public PlotImpl
		{
		public:
			CocoaPlotImpl(const std::vector<std::pair<float, float>>& data, const std::string& title, const std::string& xLabel, const std::string& yLabel);
			~CocoaPlotImpl() override;
			void Show() override;
			bool BrowseForFolder(std::string& outFolder) override;
			void DrawPlot(const int &newHeight, const int &newWidth);
			
			void OnResize(const int& newWidth, const int& newHeight) override;
			const std::vector<std::pair<float, float>> GetTransformedData(const int windowWidth, const int windowHeight,  const std::pair<float, float>& plotBorderOffsets);
			void OnMouseMove(int x, int y) override;
		protected:
			std::pair<float, float> GetTransformedCoordinates(const int& x,
															  const int& y,
															  const int windowWidth,
															  const int& windowHeight);
			void DrawTextAtPosition(int x, int y, const std::string& text);
			void DrawVerticalTextAtPosition(int x, int y, const std::string& text);
			void DrawVerticalTick(const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength);
			void DrawHorizontalTick(const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength);
			void DrawAxes(const std::pair<float, float>& plotBorderOffsets);
			void SetPlotViewData(const std::vector<std::pair<float, float>>& plotPoints);
			void SetPlotViewMouseCoords(const float& x, const float& y);
			
			PlotView* plotView;
			id resizeObserver;
			id mouseMoveObserver;
		};
#endif
#endif

		// Pointer to Plotter implementation
		static std::unique_ptr<PlotImpl> PlotImpl_;
	};
	
	// Initialize static plotter implementation
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
		// These values will be used to adjust the data points during transformation to achieve this.
		PlotZeroOffset.first = smallestX;
		PlotZeroOffset.second = smallestY;
	}

	inline std::string Plot2D::PlotImpl::GetTimestamp()
	{
		auto now = std::chrono::system_clock::now();
		std::time_t now_time = std::chrono::system_clock::to_time_t(now);
		std::string timestamp;

		std::tm local_time;
#if defined(_WIN32)
	localtime_s(&timeinfo, &now_time);
#else
	localtime_r(&now_time, &local_time);
#endif
		std::ostringstream oss;
		oss << std::put_time(&local_time, "%Y%m%d%H%M%S");
		timestamp = oss.str();

		return timestamp;
	}

#pragma region Win32
#ifdef _WIN32
	inline Plot2D::Win32PlotImpl::Win32PlotImpl(const std::vector<std::pair<float, float>>& data,
		const std::string& title, const std::string& xLabel, const std::string& yLabel) :
PlotImpl(data, title, xLabel, yLabel)
	{
		Gdiplus::GdiplusStartupInput gdiplusStartupInput;
		ULONG_PTR gdiplusToken;
		InitGDIPlus(gdiplusStartupInput, gdiplusToken);
	}
	inline Plot2D::Win32PlotImpl:~Win32PlotImpl()
	{
		ShutdownGDIPlus(gdiplusToken);
	}
	void Plot2D::Win32PlotImpl::Show()
	{
//        Gdiplus::GdiplusStartupInput gdiplusStartupInput;
//        ULONG_PTR gdiplusToken;
//        InitGDIPlus(gdiplusStartupInput, gdiplusToken);

		HWND hwnd = CreatePlotWindow(NULL, title);

		ShowWindow(hwnd, SW_SHOW); //TODO: need nCmdShow?
		UpdateWindow(hwnd);

		MSG msg = {};
		while (GetMessage(&msg, NULL, 0, 0))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

//        ShutdownGDIPlus(gdiplusToken);

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

	void Plot2D::Win32PlotImpl::DrawVerticalTextAtPosition(HDC hdc, int x, int y, const std::string& text)
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
			int x = static_cast<int>(((point.first - PlotZeroOffset.first) * xScale) + plotBorderOffsets.first);
			int y = static_cast<int>(yPlotSpan - ((point.second - PlotZeroOffset.second) * yScale) + plotBorderOffsets.second);

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
		std::string labelStr = label.str();
		SIZE textSize;
		TEXTMETRIC tm;
		GetTextMetrics(hdc, &tm);
		int avgCharWidth = tm.tmAveCharWidth;
		int textWidth = avgCharWidth * (labelStr.size() + 1);
		DrawTextAtPosition(hdc, max(10, tickCenterX - textWidth - tickLength), tickCenterY - 4, labelStr);
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

		std::wstringstream label;

		// Draw X-axis ticks
		int numTicksX = 4;
		int x = 0;
		float offset = PlotZeroOffset.first;
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
		offset = PlotZeroOffset.second;
		// Reverse loop since Windows set origin at upper left
		for (int i = numTicksY; i > 0; i--)
		{
			y = topBorderPos + (i * (bottomBorderPos - topBorderPos) / (numTicksY + 1));
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

		// Set text color and background color
		SetTextColor(hdc, PLTCOLOR_WHITE);
		RECT coordinateRect = PLTRECT_MOUSE_DISPLAY_COORDS;
		HBRUSH blackBrush = CreateSolidBrush(PLTCOLOR_BLACK);
		FillRect(hdc, &coordinateRect, blackBrush);
		DeleteObject(blackBrush);
		SetBkMode(hdc, TRANSPARENT);
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
			defaultWindowSize.first,
			defaultWindowSize.second,
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

			if (datasetSize > PLOT_CONTINUOUS_DRAW_LIMIT && isResizing)
			{
				// Dont paint while resizing for large datasets to reduce lag
				return 0;
			}

			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hwnd, &ps);
			if (isMouseMoving)
			{
				DrawCoordinates(hdc, rect, mouseDisplayCoordinates);
			}
			else
			{
				DrawPlot(hdc, rect, data, plotBorderOffsets);
			}
			EndPaint(hwnd, &ps);

			return 0;
		}
		case WM_ENTERSIZEMOVE: {
			isResizing = true;
			return 0;
		}
		case WM_EXITSIZEMOVE: {
			isResizing = false;
			InvalidateRect(hwnd, NULL, TRUE);
			return 0;
		}
		case WM_SIZE:
			if (datasetSize < PLOT_CONTINUOUS_DRAW_LIMIT)
			{
				// Allow continuous redraw
				InvalidateRect(hwnd, NULL, TRUE);
				break;
			}
			else
			{
				// Dataset is too large for smooth scaling, so prevent redraw until after size is finished
				if (isResizing) return 0;
			}
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
			minMaxInfo->ptMinTrackSize.x = minWindowSize.first;
			minMaxInfo->ptMinTrackSize.y = minWindowSize.second;
			break;
		}
		case WM_MOUSEMOVE: {
			// Get the mouse position
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			bool inside = (x >= plotBorderOffsets.first && x <= rect.right - plotBorderOffsets.first) &&
				(y >= plotBorderOffsets.second && y <= rect.bottom - plotBorderOffsets.second);

			if (inside)
			{
				isMouseMoving = true;
				RECT textRect = PLTRECT_MOUSE_DISPLAY_COORDS;
				mouseDisplayCoordinates = GetMouseCoordinatesInDataSpace(rect, x, y, plotBorderOffsets);
				InvalidateRect(hwnd, &textRect, FALSE);
				UpdateWindow(hwnd);
			}
			else
			{
				isMouseMoving = false;
			}

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
	std::pair<float, float> Plot2D::Win32PlotImpl::GetMouseCoordinatesInDataSpace(RECT rect, const int x,
																				  const int y, const std::pair<float, float>& plotBorderOffset)
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
			return std::pair<float, float>((x - plotBorderOffset.first) * xScale + PlotZeroOffset.first,
				((yPlotSpan - y + plotBorderOffset.second) * yScale + PlotZeroOffset.second));
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

#pragma region APPLE
#ifdef __APPLE__
#ifdef __OBJC__
inline Plot2D::CocoaPlotImpl::CocoaPlotImpl(const std::vector<std::pair<float, float>>& data,
											const std::string& title,
											const std::string& xLabel,
											const std::string& yLabel) : PlotImpl(data, title, xLabel, yLabel)
{
}
inline Plot2D::CocoaPlotImpl::~CocoaPlotImpl()
{
	// Remove event bindings
	[[NSNotificationCenter defaultCenter] removeObserver:resizeObserver];
	[[NSNotificationCenter defaultCenter] removeObserver:mouseMoveObserver];
}
inline void Plot2D::CocoaPlotImpl::Show()
{
		if (![NSApplication sharedApplication]) {
			[NSApplication sharedApplication];
		}
		
		// Create a window
	NSRect frame = NSMakeRect(0, 0, defaultWindowSize.first, defaultWindowSize.second);
		NSWindow *window = [[NSWindow alloc] initWithContentRect:frame
													   styleMask:(NSWindowStyleMaskTitled |
																   NSWindowStyleMaskClosable |
																   NSWindowStyleMaskResizable)
														 backing:NSBackingStoreBuffered
														   defer:NO];

		[window setTitle:[NSString stringWithUTF8String:title.c_str()]];
		[window makeKeyAndOrderFront:nil];
		[window setAcceptsMouseMovedEvents:YES];
		// Create and attach a delegate to handle window close
		WindowDelegate *delegate = [[WindowDelegate alloc] init];
		[window setDelegate:delegate];
		
		NSMenu *mainMenu = [[NSMenu alloc] init];

		// Create "File" menu
		NSMenuItem *fileMenuItem = [[NSMenuItem alloc] initWithTitle:@"File" action:nil keyEquivalent:@""];
		[mainMenu addItem:fileMenuItem];

		NSMenu *fileMenu = [[NSMenu alloc] initWithTitle:@"File"];
		[fileMenuItem setSubmenu:fileMenu];

		// Add "Save" option to File menu
		NSMenuItem *saveItem = [[NSMenuItem alloc] initWithTitle:@"Save..."
														  action:@selector(savePlot:)
												   keyEquivalent:@"s"];
		[saveItem setTarget:plotView]; // Ensure it calls a method in PlotView
		[fileMenu addItem:saveItem];

		// Set the main menu
		[NSApp setMainMenu:mainMenu];
	
		// Create the PlotView instance and set it as the window's content view
		plotView = [[PlotView alloc] initWithFrame:frame];
		[plotView setNeedsDisplay:YES];
		[window setContentView:plotView];
		[plotView display];
	
		// Perform initial calculation for starting window size
	const std::pair<float, float> plotBorderOffset = {defaultWindowSize.first * PLTBORDER_OFFSET_FACTOR,
		defaultWindowSize.second * PLTBORDER_OFFSET_FACTOR};
	std::vector<std::pair<float, float>> plotPoints = GetTransformedData(defaultWindowSize.first,
																		 defaultWindowSize.second,
																		 plotBorderOffset);
	SetPlotViewData(plotPoints);
		
	DrawAxes(plotBorderOffset);
	
	// Subscribe to resize event if limit is high enough
	if(datasetSize < deferredResizeLimit)
	{
		resizeObserver = [[NSNotificationCenter defaultCenter] addObserverForName:NSWindowDidResizeNotification
																		   object:nil
																			queue:[NSOperationQueue mainQueue]
																	   usingBlock:^(NSNotification *note){
			//NSWindow* window = (NSWindow*)note.object;
			NSSize size = plotView.frame.size;
			this->OnResize(size.width, size.height);
		}];
	}
	else
	{
		resizeObserver = [[NSNotificationCenter defaultCenter] addObserverForName:NSWindowDidEndLiveResizeNotification
																		   object:nil
																			queue:[NSOperationQueue mainQueue]
																	   usingBlock:^(NSNotification *note){
			//NSWindow* window = (NSWindow*)note.object;
			NSSize size = plotView.frame.size;
			this->OnResize(size.width, size.height);
		}];
	}
	
		// Subscribe to mouse events
		mouseMoveObserver = [[NSNotificationCenter defaultCenter] addObserverForName:MouseMovedNotification
																			  object:nil
																			   queue:[NSOperationQueue mainQueue]
																		  usingBlock:^(NSNotification *note){
			
				NSDictionary *userInfo = note.userInfo;
				float x = [userInfo[@"x"] floatValue];
				float y = [userInfo[@"y"] floatValue];
				this->OnMouseMove(x, y);
		
		}];
	
		// Start the event loop
		[NSApp run];
		
}
void Plot2D::CocoaPlotImpl::OnMouseMove(int x, int y)
{
	CGSize size = plotView.frame.size;
	std::pair<float, float> plotCoords = GetTransformedCoordinates(x, y, size.width, size.height);
	SetPlotViewMouseCoords(plotCoords.first, plotCoords.second);
}

void Plot2D::CocoaPlotImpl::SetPlotViewMouseCoords(const float& x, const float& y)
{
	// Convert C++ vector to NSPoint for Objective-C
	NSPoint point = NSMakePoint(static_cast<CGFloat>(x),
								static_cast<CGFloat>(y));
	
	[plotView updateDisplayCoordinates:point];
	
}
std::pair<float, float> Plot2D::CocoaPlotImpl::GetTransformedCoordinates(const int& x,const int& y,
																		 const int windowWidth, const int& windowHeight)
{
	const std::pair<float, float> plotBorderOffset = {windowWidth* PLTBORDER_OFFSET_FACTOR,
		windowHeight * PLTBORDER_OFFSET_FACTOR};
	if ((x >= plotBorderOffset.first && x <= windowWidth - plotBorderOffset.first) &&
		(y >= plotBorderOffset.second && y <= windowHeight - plotBorderOffset.second))
	{
		// Scale absolute mouse coordinates to plot window space
		const float xPlotSpan = windowWidth - 2 * plotBorderOffset.first;
		const float yPlotSpan = windowHeight - 2 * plotBorderOffset.second;
		float xScale = dataSpanX / xPlotSpan;
		float yScale = dataSpanY / yPlotSpan;

		// In Cocoa windows, (0,0) is botton left corner
		return std::pair<float, float>((x - plotBorderOffset.first) * xScale + PlotZeroOffset.first,
			((/*yPlotSpan - */y - plotBorderOffset.second) * yScale + PlotZeroOffset.second));
	}
	else
	{
		// Outside the plot window just default to (0,0)
		return std::pair<float, float>(0.0f, 0.0f);
	}
}
const std::vector<std::pair<float, float>> Plot2D::CocoaPlotImpl::GetTransformedData(const int windowWidth, const int windowHeight, const std::pair<float, float>& plotBorderOffsets)
{
	
	const float xPlotSpan = windowWidth - 2 * plotBorderOffsets.first;
	const float yPlotSpan = windowHeight - 2 * plotBorderOffsets.second;
	const float xScale = xPlotSpan / dataSpanX;
	const float yScale = yPlotSpan / dataSpanY;

	
	
//	size_t numThreads = std::thread::hardware_concurrency();
//	size_t chunkSize = data.size() / numThreads;
//	std::vector<std::thread> threads;
//	for (size_t i = 0; i < numThreads; ++i) {
//			size_t start = i * chunkSize;
//			size_t end = (i == numThreads - 1) ? data.size() : start + chunkSize;
//			threads.emplace_back(processChunk, std::ref(data), start, end);
//		}
	
	std::vector<std::pair<float, float>> result(datasetSize);

	// MacOs windows have origin at bottom left, so don't need to flip coordinates
	std::transform(/*std::execution::par,*/ data.begin(), data.end(), result.begin(),
				   [this, plotBorderOffsets, xScale, &yPlotSpan, yScale](const std::pair<float, float>& point) -> std::pair<float, float> {
				return { (((point.first - PlotZeroOffset.first) * xScale) + plotBorderOffsets.first),
					((point.second - PlotZeroOffset.second) * yScale) + plotBorderOffsets.second };
			}
		);
	
	return result;
}
void Plot2D::CocoaPlotImpl::DrawPlot(const int &newHeight, const int &newWidth) {
	const std::pair<float, float> plotBorderOffsets = {newWidth* PLTBORDER_OFFSET_FACTOR,
		newHeight * PLTBORDER_OFFSET_FACTOR};
	std::vector<std::pair<float, float>> plotPoints = GetTransformedData(newWidth, newHeight, plotBorderOffsets);
	SetPlotViewData(plotPoints);
	DrawAxes(plotBorderOffsets);
	[plotView draw];
}

//void Plot2D::CocoaPlotImpl::ProcessChunk(std::vector<std::pair<float, float>>& result, size_t start, size_t end, const std::pair<float, float>& plotSpan, const std::pair<float,float>& scalar)
//{
//	std::transform(data.begin(), data.end(), result.begin(),
//				   [this, plotBorderOffsets, scalar, plotSpan](const std::pair<float, float>& point) -> std::pair<float, float> {
//				return { (((point.first - PlotZeroOffset.first) * scalar.first) + plotBorderOffsets.first), plotSpan.second - ((point.second - PlotZeroOffset.second) * scalar.second) + plotBorderOffsets.second };
//			}
//		);
//}
void Plot2D::CocoaPlotImpl::OnResize(const int& newWidth, const int& newHeight)
{
	DrawPlot(newHeight, newWidth);
}
void Plot2D::CocoaPlotImpl::SetPlotViewData(const std::vector<std::pair<float, float>>& plotPoints)
{
	// Convert C++ vector to NSArray<NSValue *> for Objective-C
	NSMutableArray<NSValue *> *dataArray = [NSMutableArray array];
	for (const auto &point : plotPoints)
	{
		NSPoint nsPoint = NSMakePoint(point.first, point.second);
		[dataArray addObject:[NSValue valueWithPoint:nsPoint]];
	}
	
	[plotView updateData:dataArray];
}

bool Plot2D::CocoaPlotImpl::BrowseForFolder(std::string& outFolder)
{
	return true;
}

void Plot2D::CocoaPlotImpl::DrawTextAtPosition(int x, int y, const std::string &text)
{
	[plotView addText:[NSString stringWithUTF8String:text.c_str()]
		   atPosition:NSMakePoint(x, y)];
}

void Plot2D::CocoaPlotImpl::DrawVerticalTextAtPosition(int x, int y, const std::string &text)
{
	
}

void Plot2D::CocoaPlotImpl::DrawVerticalTick(const int tickCenterX, const int tickCenterY,
											 const float tickValue, const int tickLength)
{
	
	// Draw line
	[plotView addLineFrom:NSMakePoint(tickCenterX, tickCenterY - tickLength)
					   to:NSMakePoint(tickCenterX, tickCenterY + tickLength)];
	// Add label
	std::stringstream label;
	label << std::setprecision(4) << tickValue;

	[plotView addText:[NSString stringWithUTF8String:label.str().c_str()]
		   atPosition:NSMakePoint(tickCenterX - 10, fmax(tickCenterY - 30, 10))];

}

void Plot2D::CocoaPlotImpl::DrawHorizontalTick(const int tickCenterX, const int tickCenterY, const float tickValue, const int tickLength) {
	
	// Draw line
	[plotView addLineFrom:NSMakePoint(tickCenterX - tickLength, tickCenterY)
					   to:NSMakePoint(tickCenterX + tickLength, tickCenterY)];
	// Add label
	std::stringstream label;
	label << std::setprecision(4) << tickValue;
	
	NSDictionary *attributes = @{
		NSFontAttributeName: [NSFont systemFontOfSize:10]
	};
	NSString* text = [NSString stringWithUTF8String:label.str().c_str()];
	// Get the text size
	NSSize textSize = [text sizeWithAttributes:attributes];
	CGFloat textWidth = textSize.width;
	
	[plotView addText:text
		   atPosition:NSMakePoint(fmax(10, tickCenterX - textWidth - tickLength-8), tickCenterY - 6)];;
}

void Plot2D::CocoaPlotImpl::DrawAxes(const std::pair<float, float> &plotBorderOffsets) {
	
	[plotView.lines removeAllObjects];
	[plotView.textEntries removeAllObjects];
	Rect2D windowRect(plotView.frame.size.height, 0, 0, plotView.frame.size.width);
	const int leftBorderPos = static_cast<int>(plotBorderOffsets.first);
	const int rightBorderPos = static_cast<int>(windowRect.right - plotBorderOffsets.first);
	const int topBorderPos = static_cast<int>(windowRect.top - plotBorderOffsets.second);
	const int bottomBorderPos = static_cast<int>(plotBorderOffsets.second);

	// Draw the border
	[plotView addLineFrom:NSMakePoint(leftBorderPos, bottomBorderPos) to:NSMakePoint(leftBorderPos, topBorderPos)];
	[plotView addLineFrom:NSMakePoint(leftBorderPos, bottomBorderPos) to:NSMakePoint(rightBorderPos, bottomBorderPos)];

	// Draw labels
	// TODO: use plot props
	DrawTextAtPosition(windowRect.right / 2, 20, xLabel);
	//DrawVerticalTextAtPosition(windowRect.left + 10, windowRect.top / 2, yLabel);
	
	// Draw X-axis ticks
	int numTicksX = 4;
	int x = 0;
	float offset = PlotZeroOffset.first;
	float increment = dataSpanX / (numTicksX + 1);
	for (int i = 1; i <= numTicksX; ++i)
	{
		x = leftBorderPos + i * (rightBorderPos - leftBorderPos) / (numTicksX + 1);
		offset += increment;
		DrawVerticalTick(x, bottomBorderPos, offset, tickLength);
	}
	DrawVerticalTick(rightBorderPos, bottomBorderPos, offset + increment, tickLength);
	
	// Draw Y-axis ticks
	int numTicksY = 4;
	int y = 0;
	increment = dataSpanY / (numTicksY + 1);
	offset = PlotZeroOffset.second;
		for (int i = 1; i <= numTicksY; i++)
	{
		y = bottomBorderPos + i * ((topBorderPos - bottomBorderPos) / (numTicksY + 1));
		offset += increment;
		DrawHorizontalTick(leftBorderPos, y, offset, tickLength);
	}
	DrawHorizontalTick(leftBorderPos, topBorderPos, offset + increment, tickLength);
	
}



#endif
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
#ifdef __OBJC__
		PlotImpl_ = std::make_unique<CocoaPlotImpl>(points, title, xLabel, yLabel);
#endif
#elif defined(__linux__)
		PlotImpl_ = std::make_unique<X11PlotImpl>(points, title, xLabel, yLabel);
#else
		std::throw(std::exception("Unknown platform."));
#endif

		PlotImpl_->Show();
		
		PlotImpl_.reset();

	}

	// Explicit template instantiation for common numeric types
	template void Plot2D::Plot<int>(const std::vector<int>& x, const std::vector<int>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<float>(const std::vector<float>& x, const std::vector<float>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<double>(const std::vector<double>& x, const std::vector<double>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<long>(const std::vector<long>& x, const std::vector<long>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);
	template void Plot2D::Plot<short>(const std::vector<short>& x, const std::vector<short>& y, const std::string& title, const std::string& xLabel, const std::string& yLabel);

} // cxpplot

