#pragma once

#pragma once
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cwchar>
#include <execution>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <map>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <functional>
#include <vector>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#include <gdiplus.h>  // This must be inlcuded after windows.h
#include <shlobj.h>
#pragma comment(lib, "gdiplus.lib")
#elif defined(__linux__)
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#elif defined(__APPLE__)
#ifdef __OBJC__
#import <Cocoa/Cocoa.h>
#import <Foundation/Foundation.h>

#pragma mark - ObjC

// Need to define ObjC classes in global namespace
@interface PlotView : NSView
@property(nonatomic, copy) NSArray<NSValue*>* dataPoints;
@property(nonatomic) NSPoint mouseCoordinates;
@property(nonatomic, strong) NSMutableArray<NSDictionary*>* textEntries;
@property(nonatomic, strong) NSMutableArray<NSValue*>* lines;
@property(nonatomic, strong) NSTrackingArea* trackingArea;
@property(nonatomic, assign) BOOL onlyDrawCoordinates;
@end
extern NSString* const MouseMovedNotification = @"MouseMovedNotification";
@implementation PlotView
- (instancetype)initWithFrame:(NSRect)frame
{
    self = [super initWithFrame:frame];
    if (self)
    {
        [self.window setAcceptsMouseMovedEvents:YES];
        [self addTrackingArea:[[NSTrackingArea alloc]
                                  initWithRect:self.bounds
                                       options:(NSTrackingMouseMoved | NSTrackingActiveInKeyWindow)
                                         owner:self
                                      userInfo:nil]];
        _lines = [[NSMutableArray alloc] init];
        _onlyDrawCoordinates = false;
    }
    return self;
}

- (void)updateTrackingAreas
{
    [super updateTrackingAreas];

    // Remove any existing tracking areas
    if (self.trackingArea)
    {
        [self removeTrackingArea:self.trackingArea];
    }

    // Create a new tracking area
    NSTrackingArea* trackingArea =
        [[NSTrackingArea alloc] initWithRect:self.bounds
                                     options:(NSTrackingMouseEnteredAndExited |
                                              NSTrackingMouseMoved | NSTrackingActiveInKeyWindow)
                                       owner:self
                                    userInfo:nil];
    [self addTrackingArea:trackingArea];
    self.trackingArea = trackingArea;
}
- (void)mouseMoved:(NSEvent*)event
{
    NSPoint location = [self convertPoint:event.locationInWindow fromView:nil];

    // Post notification with mouse position
    NSDictionary* userInfo = @{@"x" : @(location.x), @"y" : @(location.y)};
    [[NSNotificationCenter defaultCenter] postNotificationName:MouseMovedNotification
                                                        object:self
                                                      userInfo:userInfo];

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
- (void)resetCursorRects
{
    [self addCursorRect:[self bounds] cursor:[NSCursor resizeLeftRightCursor]];
}
- (void)DrawMouseCoordinates
{
    if (!CGPointEqualToPoint(self.mouseCoordinates, NSZeroPoint))
    {
        // Format the coordinates as a string
        NSString* coordinateText = [NSString
            stringWithFormat:@"X=%.2f  Y=%.2f", self.mouseCoordinates.x, self.mouseCoordinates.y];

        // Set attributes for the text
        NSDictionary* mouseCoordAttr = @{
            NSFontAttributeName : [NSFont systemFontOfSize:12],
            NSForegroundColorAttributeName : [NSColor whiteColor]
        };

        // Draw the formatted string at a top left of the window
        NSPoint drawPoint = NSMakePoint(10, self.bounds.size.height - 20);
        [coordinateText drawAtPoint:drawPoint withAttributes:mouseCoordAttr];
    }
}

- (void)DrawPlot
{
    // Set color for the axes
    [[NSColor whiteColor] setStroke];

    // Init current graphics context
    CGContextRef context = [[NSGraphicsContext currentContext] CGContext];
    CGContextSetStrokeColorWithColor(context, [[NSColor blackColor] CGColor]);
    CGContextSetLineWidth(context, 1.0);

    // Draw axis labels
    NSDictionary* axisLabelAttr = @{
        NSFontAttributeName : [NSFont systemFontOfSize:10],
        NSForegroundColorAttributeName : [NSColor whiteColor]
    };

    for (NSDictionary* entry in self.textEntries)
    {
        NSString* text = entry[@"text"];
        NSPoint position = [entry[@"position"] pointValue];

        [text drawAtPoint:position withAttributes:axisLabelAttr];
    }

    // Draw axis and tick lines
    for (NSUInteger i = 0; i < self.lines.count; i += 2)
    {
        NSPoint start, end;
        [self.lines[i] getValue:&start size:sizeof(NSPoint)];
        [self.lines[i + 1] getValue:&end size:sizeof(NSPoint)];

        CGContextMoveToPoint(context, start.x, start.y);
        CGContextAddLineToPoint(context, end.x, end.y);
    }
    // Draw all strokes at the same time. Need to call this for each color
    CGContextStrokePath(context);

    // Draw lines to each point
    [[NSColor greenColor] setStroke];  // Set color for the plot line
    NSPoint point = [[self.dataPoints objectAtIndex:0] pointValue];
    CGContextMoveToPoint(context, point.x, point.y);
    for (NSValue* pointValue in self.dataPoints)
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

    // Skip redrawing anything else this pass if the user is just moving their mouse
    if (self.onlyDrawCoordinates)
    {
        return;
    }

    [self DrawPlot];
}

- (void)addLineFrom:(NSPoint)start to:(NSPoint)end
{
    NSValue* line = [NSValue valueWithBytes:&start objCType:@encode(NSPoint)];
    NSValue* endLine = [NSValue valueWithBytes:&end objCType:@encode(NSPoint)];

    [self.lines addObject:line];
    [self.lines addObject:endLine];
}
- (void)draw
{
    [self setNeedsDisplay:YES];  // Trigger a full redraw
}
- (void)updateData:(NSArray<NSValue*>*)newData
{
    self.dataPoints = newData;
}
- (void)updateDisplayCoordinates:(NSPoint)newCoords
{
    self.mouseCoordinates = newCoords;

    NSRect dirtyRect = NSMakeRect(0, self.bounds.size.height - 20, 200, 30);
    // Flag used in draw call to only render mouse coordinates.
    // No need to redraw all lines if the user is just moving their mouse over the plot
    self.onlyDrawCoordinates = true;

    // Mark only the small rect for redraw
    [self setNeedsDisplayInRect:dirtyRect];
}
- (void)addText:(NSString*)text atPosition:(NSPoint)position
{
    if (!self.textEntries)
    {
        self.textEntries = [NSMutableArray array];
    }

    NSDictionary* textEntry = @{@"text" : text, @"position" : [NSValue valueWithPoint:position]};

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
    self.onlyDrawCoordinates = false;  // Resizing, so we need to redraw whole plot
    [self setNeedsDisplay:YES];
}
- (void)savePlot:(id)sender
{
    // Show a save dialog
    NSSavePanel* savePanel = [NSSavePanel savePanel];
    NSArray* fileTypes = [NSArray arrayWithObjects:@"png", nil];
    savePanel.allowedContentTypes = fileTypes;

    [savePanel setNameFieldStringValue:@"plot.png"];

    if ([savePanel runModal] == NSModalResponseOK)
    {
        NSURL* saveURL = [savePanel URL];

        // Render view into an image
        NSBitmapImageRep* imageRep = [self bitmapImageRepForCachingDisplayInRect:self.bounds];
        [self cacheDisplayInRect:self.bounds toBitmapImageRep:imageRep];

        // Convert to PNG m_data
        NSData* pngData =
            [imageRep representationUsingType:NSBitmapImageFileTypePNG properties:@{}];

        // Write to file
        [pngData writeToURL:saveURL atomically:YES];
    }
}
@end

// Window delegate to handle close event
@interface WindowDelegate : NSObject <NSWindowDelegate>
@end

@implementation WindowDelegate

- (void)windowWillClose:(NSNotification*)notification
{
    [NSApp stop:nil];  // Stops the event loop when the window closes
}

@end
#pragma mark
#pragma mark - End of ObjC
#endif
#endif

namespace cpplot2d
{

struct Color
{
    uint8_t r = 0.f;
    uint8_t g = 0.f;
    uint8_t b = 0.f;
    uint8_t a = 1.f;  // alpha, default opaque

    constexpr Color() = default;
    constexpr Color(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha = 1)
        : r(red), g(green), b(blue), a(alpha)
    {
    }

    // Common predefined colors
    static constexpr Color Black()
    {
        return {0, 0, 0};
    }
    static constexpr Color White()
    {
        return {255, 255, 255};
    }
    static constexpr Color Red()
    {
        return {255, 0, 0};
    }
    static constexpr Color Green()
    {
        return {0, 255, 0};
    }
    static constexpr Color Blue()
    {
        return {0, 0, 255};
    }

    static constexpr Color FromRGB(int red, int green, int blue, int alpha = 255)
    {
        return {static_cast<uint8_t>(red), static_cast<uint8_t>(green), static_cast<uint8_t>(blue),
                static_cast<uint8_t>(alpha)};
    }

    constexpr bool operator<(const Color& other) const noexcept
    {
        if (r != other.r) return r < other.r;
        if (g != other.g) return g < other.g;
        if (b != other.b) return b < other.b;
        return a < other.a;
    }

    constexpr bool operator==(const Color& other) const noexcept
    {
        return r == other.r && g == other.g && b == other.b && a == other.a;
    }
};
class Plot2D
{
   public:
    template <typename T>
    Plot2D(const std::vector<T>& x, const std::vector<T>& y, const std::string& title = "Plot",
           const std::string& xLabel = "x", const std::string& yLabel = "y");

    /**
     Show the plot with the pre-determined plot points and parameters.
     Set block to false to prevent blocking the main thread (e.g. when created from within another gui app).
     */
    void Show(bool block = true);

    /**
     Sets whether the legend should be shown on the plot or not.

     @param show whether or not to show the legend
     */
    void DisplayLegend(bool show);

    /**
     Sets whether or not to enable multithreading for m_data processing.

     @param enable enables multithreading if applicable
     */
    void SetMultithreadingEnabled(bool enable);

    /**
     Sets the bottom limit for deferring window redraws until the window is idle. Decrease this
     value if resizing the window causes flickering.

     @param limit the number of plot points beyond which the window will deferr redrawing until it
     is in an idle state.
     */
    void SetDeferredDrawLimit(int limit = 5000);

   protected:
    using Point = std::pair<int, int>;

    struct WindowRect
    {
       public:
        WindowRect(int top, int left, int right, int bottom)
            : top(top), bottom(bottom), right(right), left(left)
        {
        }

        std::pair<int, int> Size()
        {
            return {right - left, top - bottom};
        }

        int top;
        int left;
        int right;
        int bottom;
    };

    enum class Orientation : uint8_t
    {
        HORIZONTAL = 0,
        VERTICAL = 1
    };

    class IDGenerator
    {
       public:
        static int Next()
        {
            static int current = 1000;
            return current++;
        }
    };

    class FileName
    {
       public:
        static std::string Create(const std::string& dir, const std::string& filename);
    };

    struct GuiPoints
    {
        Point p;
        int size;
    };

    // TODO: Can make this a Ployline that has a vector<points>
    // that way we dont copy the color a million times for the same set of lines
    struct GuiPolyline
    {
       public:
        GuiPolyline(std::vector<Point> points, Color c, int thickness = 1)
            : points(points), color(c), thickness(thickness)
        {
        }
        std::vector<Point> points;
        Color color;
        int thickness;
    };
    struct GuiLine
    {
       public:
        GuiLine(Point p1, Point p2, Color c) : p1(p1), p2(p2), color(c)
        {
        }
        Point p1, p2;
        Color color;
    };
    struct GuiText
    {
       public:
        GuiText(const std::string& text, const Point pos, int spacing, const Color& color,
                Orientation orientation)
            : text(text), pos(pos), spacing(spacing), color(color), orientation(orientation)
        {
        }
        GuiText() = default;
        std::string text = "";
        Point pos = {0,0};
        int spacing = 1;
        Color color = Color::White();
        Orientation orientation = Orientation::HORIZONTAL;
    };
    struct GuiRect
    {
        Point topLeft;
        Point bottomRight;
        Color fillColor;
        Color borderColor;
        int borderWidth;
    };

    struct GuiCircle
    {
        Point center;
        int radius;
        Color fillColor;
        Color borderColor;
        int borderWidth;
    };

    struct GuiPolygon
    {
        std::vector<Point> vertices;
        Color fillColor;
        Color borderColor;
        int borderWidth;
    };

    struct WindowState
    {
        std::vector<GuiText> text;
        std::vector<GuiLine> lines;
        std::vector<GuiPolyline> polylines;
        std::vector<GuiPoints> points;
        std::vector<GuiRect> rects;
        std::vector<GuiCircle> circles;
        std::vector<GuiPolygon> polygons;
        Color background;
        std::pair<int, int> minSize;
    };

    class IWindow
    {
       public:

        // Draws text at given x and y position in window space. Returns id to that text
        virtual void DrawTextAt(const std::string text, int spacing, int size = 1,
                                Point startPos = {0, 0},
                                Orientation orientation = Orientation::HORIZONTAL,
                                Color color = Color::White()) = 0;

        // Invalidates entire window, forcing a redraw
        virtual void Invalidate(const WindowState& windowState) = 0;
        virtual int GetAverageCharWidth() = 0;
        virtual void DrawWindowState() = 0;

        // Invalidates only the space  encompassed in the given rect
        virtual void InvalidateRegion(const WindowRect& rect, const WindowState& windowState) = 0;
        virtual void AddMenuButton(const std::string menu, const std::string label,
                                   std::function<void()> onClickCallback) = 0;
        virtual bool SaveScreenshotAsPNG(const std::string& fileName) = 0;
        virtual void SetIsVisible(bool isVisible) = 0;
        virtual std::string GetTimestamp() = 0;
        virtual WindowRect GetRect() = 0;
        virtual void RunEventLoop() = 0;
        std::function<void(Point)> OnMouseHoverCallback;
        std::function<void()> OnResizeStartCallback;
        std::function<void()> OnResizeEndCallback;
        std::function<void()> OnResizeCallback;
    };
    class IGraphicsContext
    {
       public:
        virtual void Init() = 0;
        virtual void Shutdown() = 0;
        virtual std::unique_ptr<Plot2D::IWindow> MakeWindow(Color background,
                                                            std::pair<int, int> defaultSize,
                                                            std::pair<int, int> minSize,
                                                            bool isVisible,
                                                            const std::string& title) = 0;

       protected:
        ULONG_PTR m_gdiplusToken;
    };

    // TODO: USE
    struct PlotProperties
    {
        Color backgroundColor;
        Color borderColor;
        bool showLegend;
        int tickLineCount;
        int labelTextSpacing;
        std::string plotTitle;
        std::string plotXLabel;
        std::string plotYLabel;
    };

    static std::unique_ptr<IGraphicsContext> m_graphicsContext;
    std::unique_ptr<IWindow> m_window;
    void OnMouseHoverCallback(IWindow& window, Point mousePos);
    void OnWindowResizeCallback(IWindow& window);
    void OnSaveButtonClicked(IWindow& window);
    void OnWindowResizeEndCallback(IWindow& window);
    std::pair<float, float> GetPlotBorderOffsets();
    void SetPlotBorderOffsets(std::pair<float, float> offsets);
    std::pair<float, float> Plot2D::GetTransformedCoordinates(const int& x, const int& y,
                                                              const int windowWidth,
                                                              const int& windowHeight);
    void UpdatePlotWindowState(WindowState& windowState, IWindow& window);
    // Returns data points as polyline based on window sizes
    // TOOD: account for zoom here?
    GuiPolyline GetDataPolyline(const std::vector<std::pair<float, float>>& data, Color color,
                                IWindow& window);

    // Returns plot border as polyline
    GuiPolyline GetPlotBorderPolyline(IWindow& window, Color color);
    std::pair<std::vector<GuiLine>, std::vector<GuiText>> GetPlotBorderTickLines(IWindow& window,
                                                                                 Color color);
    std::vector<GuiText> GetPlotLabels(IWindow& window, Color color);

    std::pair<float, float> AbsToPlot(IWindow& window, const int x, const int y,
                                      const std::pair<float, float>& plotBorderOffset);

    std::vector<std::pair<float, float>> m_data;
    std::pair<float, float> PlotZeroOffset;
    std::pair<float, float> m_plotBorderOffsets;
    const float m_plotBorderOffsetFactor = 0.105;
    std::string xLabel;
    std::string yLabel;
    std::string title;
    float dataSpanX = 0;
    float dataSpanY = 0;
    size_t datasetSize = 0;
    const int tickLength = 4;
    const std::pair<int, int> minWindowSize = {200, 100};
    std::pair<int, int> defaultWindowSize = {800, 600};
    int m_deferredResizeLimit = 1000000;
    int m_mouseCoordTextId;
    WindowState m_plotWindowState;

#ifdef _WIN32
    class Win32GraphicsContext : public Plot2D::IGraphicsContext
    {
       public:
        void Init() override;
        void Shutdown() override;
        virtual std::unique_ptr<Plot2D::IWindow> MakeWindow(Color background,
                                                            std::pair<int, int> defaultSize,
                                                            std::pair<int, int> minSize,
                                                            bool isVisible,
                                                            const std::string& title) override;
    };
    class Win32Window : public Plot2D::IWindow
    {
       public:
        Win32Window(std::pair<int, int> defaultWindowSize, std::pair<int, int> pos,
                    std::string title);

        void DrawTextAt(const std::string text, int spacing, int size = 1, Point startPos = {0,0},
                        Orientation orientation = Orientation::HORIZONTAL, Color color = Color::White()) override;
        int GetAverageCharWidth() override;
        void AddMenuButton(const std::string menu, const std::string label,
                           std::function<void()> onClickCallback) override;
        void SetIsVisible(bool isVisible) override;
        void Invalidate(const WindowState& windowState) override;
        void DrawWindowState() override;
        std::string GetTimestamp() override;
        void RunEventLoop() override;
        void InvalidateRegion(const WindowRect& rect, const WindowState& windowState) override;
        // LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
        bool BrowseForFolder(std::string& outFolder);
        WindowRect GetRect() override;
        bool SaveScreenshotAsPNG(const std::string& fileName) override;
        static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

       protected:
        WindowState m_windowState;
        HWND m_hwnd;
        std::map<int, std::function<void()>> m_menuCommands;

        LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
        void SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename);
        HBITMAP CaptureWindowContent(HWND hwnd);
        COLORREF ToWin32Color(const Color color);
        void DoDrawText(HDC hdc, GuiText text, RECT clientRect);
    };

#elif defined(__linux__)

#elif defined(__APPLE__)

#endif

};  // Plot2D

#ifdef _WIN32

std::string Plot2D::FileName::Create(const std::string& dir, const std::string& filename)
{
    std::string safeFilename = filename;
    std::replace(safeFilename.begin(), safeFilename.end(), ':', '_');

    std::string name = dir + "\\" + safeFilename;
    return name;
}
void Plot2D::Win32Window::RunEventLoop()
{
    // Standard Win32 blocking message loop — exits when WindowProc posts PostQuitMessage
    // (WM_DESTROY)
    MSG msg;
    while (GetMessage(&msg, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}

void Plot2D::Win32GraphicsContext::Init()
{
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    Gdiplus::GdiplusStartup(&m_gdiplusToken, &gdiplusStartupInput, NULL);

}

void Plot2D::Win32GraphicsContext::Shutdown()
{
    Gdiplus::GdiplusShutdown(m_gdiplusToken);
 
}

std::unique_ptr<Plot2D::IWindow> Plot2D::Win32GraphicsContext::MakeWindow(
    Color background, std::pair<int, int> defaultSize, std::pair<int, int> minSize,
    bool isVisible, const std::string& title)
{
    return std::make_unique<Plot2D::Win32Window>(defaultSize, std::pair<int, int>(0,0), title);
}
Plot2D::Win32Window::Win32Window(std::pair<int, int> defaultWindowSize, std::pair<int, int> pos,
                                 std::string title)
{
    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = TEXT("PlotWindowClass");
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

    if (!RegisterClass(&wc))
    {
        DWORD err = GetLastError();
        // If class already exists that's fine; treat ERROR_CLASS_ALREADY_EXISTS as non-fatal
        if (err != ERROR_CLASS_ALREADY_EXISTS)
        {
            std::cout << "Failed to register window class. Error: " << err << std::endl;
        }
    }

    m_hwnd = CreateWindowEx(0, TEXT("PlotWindowClass"), title.c_str(), WS_OVERLAPPEDWINDOW,
                            CW_USEDEFAULT, CW_USEDEFAULT, defaultWindowSize.first,
                            defaultWindowSize.second, NULL, NULL, GetModuleHandle(NULL), this);

    if (!m_hwnd)
    {
        DWORD err = GetLastError();
        std::cout << "Failed to create window. Error: " << err << std::endl;
    }
}
void Plot2D::Win32Window::AddMenuButton(const std::string menu, const std::string label,
                                        std::function<void()> onClickCallback)
{
    // Create the menu
    HMENU hMenu = CreateMenu();
    HMENU hFileMenu = CreateMenu();
    int id = IDGenerator::Next();
    AppendMenu(hFileMenu, MF_STRING, id, label.c_str());
    AppendMenu(hMenu, MF_POPUP, (UINT_PTR)hFileMenu, menu.c_str());

    SetMenu(m_hwnd, hMenu);

    m_menuCommands.emplace(id, onClickCallback);
}
std::string Plot2D::Win32Window::GetTimestamp()
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
Plot2D::WindowRect Plot2D::Win32Window::GetRect()
{
    RECT rect;
    GetClientRect(m_hwnd, &rect);

    // Universal window assumes origin at bottom left. Win32 is at top left
    return WindowRect(rect.bottom, rect.left, rect.right, rect.top);
}

void Plot2D::Win32Window::SetIsVisible(bool isVisible)
{
    ShowWindow(m_hwnd, isVisible);
}

bool Plot2D::Win32Window::SaveScreenshotAsPNG(const std::string& fileName)
{
    std::string dir;
    if (BrowseForFolder(dir))
    {
        std::string name = FileName::Create(dir, fileName + ".png");
        HBITMAP hBitmap = CaptureWindowContent(m_hwnd);
        SaveHBITMAPToFile(hBitmap, name);
        DeleteObject(hBitmap);
        std::stringstream wss;
        wss << "Image Saved to" << std::string(name.begin(), name.end()) << ".";
        MessageBox(m_hwnd, wss.str().c_str(), "Saved", MB_OK);
    }
    return true;
}

HBITMAP Plot2D::Win32Window::CaptureWindowContent(HWND hwnd)
{
    RECT rect;
    if (!GetClientRect(hwnd, &rect)) return NULL;
    int width = rect.right - rect.left;
    int height = rect.bottom - rect.top;
    if (width <= 0 || height <= 0) return NULL;

    HDC hdcWindow = GetDC(hwnd);
    if (!hdcWindow) return NULL;

    HDC hdcMemDC = CreateCompatibleDC(hdcWindow);
    if (!hdcMemDC)
    {
        ReleaseDC(hwnd, hdcWindow);
        return NULL;
    }

    HBITMAP hbmScreen = CreateCompatibleBitmap(hdcWindow, width, height);
    if (!hbmScreen)
    {
        DeleteDC(hdcMemDC);
        ReleaseDC(hwnd, hdcWindow);
        return NULL;
    }

    // Select the bitmap into the mem DC and save the old one
    HBITMAP hOldBmp = (HBITMAP)SelectObject(hdcMemDC, hbmScreen);

    BOOL success = FALSE;

    // Prefer PrintWindow (asks window to paint into DC). Fallback to BitBlt.
    if (IsWindow(hwnd))
    {
        // Some windows don't support PrintWindow. It may still work better for
        // layered/double-buffered windows.
        if (PrintWindow(hwnd, hdcMemDC, PW_CLIENTONLY) == TRUE)
        {
            success = TRUE;
        }
    }

    if (!success)
    {
        // Ensure the window is up-to-date (optional; use with care)
        // UpdateWindow(hwnd);
        success = BitBlt(hdcMemDC, 0, 0, width, height, hdcWindow, 0, 0, SRCCOPY);
    }

    // Restore original bitmap into mem DC before deleting DC
    SelectObject(hdcMemDC, hOldBmp);

    DeleteDC(hdcMemDC);
    ReleaseDC(hwnd, hdcWindow);

    if (!success)
    {
        // Clean up the bitmap we created
        DeleteObject(hbmScreen);
        return NULL;
    }

    // Caller owns the HBITMAP and must DeleteObject it when done
    return hbmScreen;
}
void Plot2D::Win32Window::SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename)
{
    auto GetEncoderClsid = [](const WCHAR* mimeType, CLSID* pClsid) -> int
    {
        UINT num = 0;   // number of image encoders
        UINT size = 0;  // size of the image encoder array in bytes

        Gdiplus::GetImageEncodersSize(&num, &size);
        if (size == 0) return -1;  // Failure

        std::vector<BYTE> buffer(size);
        Gdiplus::ImageCodecInfo* pImageCodecInfo =
            reinterpret_cast<Gdiplus::ImageCodecInfo*>(buffer.data());

        Gdiplus::GetImageEncoders(num, size, pImageCodecInfo);

        for (UINT j = 0; j < num; ++j)
        {
            if (wcscmp(pImageCodecInfo[j].MimeType, mimeType) == 0)
            {
                *pClsid = pImageCodecInfo[j].Clsid;
                return static_cast<int>(j);
            }
        }
        return -1;
    };

    Gdiplus::Bitmap bitmap(hBitmap, NULL);
    CLSID clsid;
    HRESULT hr = CLSIDFromString(L"{557CF406-1A04-11D3-9A73-0000F81EF32E}", &clsid);  // PNG CLSID
    if (FAILED(hr))
    {
        // Fallback: locate encoder by MIME type
        if (GetEncoderClsid(L"image/png", &clsid) < 0)
        {
            std::wstring wfn(filename.begin(), filename.end());
            std::string msg = "Failed to obtain PNG encoder CLSID. Cannot save: " + filename;
            MessageBox(m_hwnd, msg.c_str(), "Save Error", MB_OK | MB_ICONERROR);
            return;
        }
    }
    std::wstring wstr(filename.begin(), filename.end());
    Gdiplus::Status status = bitmap.Save(wstr.c_str(), &clsid, NULL);
    if (status != Gdiplus::Ok)
    {
        std::string msg = "Failed to save image: " + filename;
        MessageBox(m_hwnd, msg.c_str(), "Save Error", MB_OK | MB_ICONERROR);
    }
}
LRESULT CALLBACK Plot2D::Win32Window::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    Win32Window* pThis = nullptr;
    if (uMsg == WM_CREATE)
    {
        CREATESTRUCT* pCreate = reinterpret_cast<CREATESTRUCT*>(lParam);
        pThis = reinterpret_cast<Win32Window*>(pCreate->lpCreateParams);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<Win32Window*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
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

bool Plot2D::Win32Window::BrowseForFolder(std::string& outFolder)
{
    BROWSEINFO bi;
    ZeroMemory(&bi, sizeof(bi));
    char szDisplayName[MAX_PATH];
    char szPath[MAX_PATH];

    bi.hwndOwner = NULL;
    bi.pidlRoot = NULL;
    bi.pszDisplayName = szDisplayName;
    bi.lpszTitle = "Choose Destination Folder";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    bi.lpfn = NULL;
    bi.lParam = 0;
    bi.iImage = 0;

    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    if (pidl != NULL)
    {
        if (SHGetPathFromIDList(pidl, szPath))
        {
            std::string result(szPath);
            CoTaskMemFree(pidl);
            outFolder = result;
            return true;
        }
        CoTaskMemFree(pidl);
    }
    return false;
}
int Plot2D::Win32Window::GetAverageCharWidth()
{
    SIZE textSize;
    TEXTMETRIC tm;
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(m_hwnd, &ps);
    GetTextMetrics(hdc, &tm);
    EndPaint(m_hwnd, &ps);
    return tm.tmAveCharWidth;
}

inline LRESULT Plot2D::Win32Window::HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam,
                                                  LPARAM lParam)
{
    RECT rect;
    GetClientRect(hwnd, &rect);

    switch (uMsg)
    {
        case WM_PAINT:
        {
            DrawWindowState();

            return 0;
        }
        case WM_ENTERSIZEMOVE:
        {
            if (OnResizeStartCallback) OnResizeStartCallback();
            return 0;
        }
        case WM_EXITSIZEMOVE:
        {
            if (OnResizeEndCallback) OnResizeEndCallback();
            InvalidateRect(hwnd, NULL, TRUE);
            return 0;
        }
        case WM_SIZE:
        {
            if (OnResizeCallback) OnResizeCallback();
            break;
        }
        case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            auto it = m_menuCommands.find(wmId);
            if (it != m_menuCommands.end())
            {
                // Invoke the function tied to this menu item
                std::function<void()> func = it->second;
                if (func) func();
            }
            break;
        }
        case WM_GETMINMAXINFO:
        {
            std::pair<int, int> sizes = m_windowState.minSize;
            MINMAXINFO* minMaxInfo = (MINMAXINFO*)lParam;
            minMaxInfo->ptMinTrackSize.x = sizes.first;
            minMaxInfo->ptMinTrackSize.y = sizes.second;
            break;
        }
        case WM_MOUSEMOVE:
        {
            // Get the mouse position
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);
            if (OnMouseHoverCallback)
                OnMouseHoverCallback(Point(
                    x, rect.bottom - y));  // Give Y in the expected universal coordinate system

            break;
        }
        case WM_SETCURSOR:
        {
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
COLORREF Plot2D::Win32Window::ToWin32Color(const Color color)
{
    return RGB(color.r, color.g, color.b);
}

void Plot2D::Win32Window::DrawTextAt(const std::string text, int spacing, int size,
                                     Point startPos,
                                     Orientation orientation,
                                     Color color)
{
    m_windowState.text.push_back(GuiText(text, startPos, spacing, color, orientation));
}
void Plot2D::Win32Window::DoDrawText(HDC hdc, GuiText text, RECT clientRect)
{
    SetTextColor(hdc, ToWin32Color(text.color));
    SetBkMode(hdc, TRANSPARENT);
    TextOut(hdc, text.pos.first, clientRect.bottom - text.pos.second, text.text.c_str(),
            static_cast<int>(text.text.length()));
}

void Plot2D::Win32Window::DrawWindowState()
{
    RECT rect;
    GetClientRect(m_hwnd, &rect);
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(m_hwnd, &ps);

    // Fill background
    HBRUSH blackBrush = CreateSolidBrush(ToWin32Color(m_windowState.background));
    FillRect(hdc, &rect, blackBrush);
    DeleteObject(blackBrush);

    // Draw Polylines
    std::map<Color, HPEN> brushes;  // HPEN hashmap
    HPEN hpen = nullptr;
    std::map<Color, HPEN>::iterator it;
    for (GuiPolyline polyline : m_windowState.polylines)
    {
        // Get brush for current line from hashmap or add new mapping
        it = brushes.find(polyline.color);
        if (it != brushes.end())
        {
            hpen = it->second;
        }
        else
        {
            hpen = CreatePen(PS_SOLID, 1, ToWin32Color(polyline.color));
            brushes.emplace(polyline.color, hpen);
        }

        int size = polyline.points.size();
        SelectObject(hdc, hpen);

        // Draw initial point, then LineTo the rest
        MoveToEx(hdc, polyline.points[0].first, rect.bottom - polyline.points[0].second, NULL);
        LineTo(hdc, polyline.points[0].first, rect.bottom - polyline.points[0].second);
        for (int i = 1; i < size; i++)
        {
            LineTo(hdc, polyline.points[i].first, rect.bottom - polyline.points[i].second);
        }
    }

    // Draw Lines
    for (GuiLine line : m_windowState.lines)
    {
        // Get brush for current line from hashmap or add new mapping

        it = brushes.find(line.color);
        if (it != brushes.end())
        {
            hpen = it->second;
        }
        else
        {
            hpen = CreatePen(PS_SOLID, 1, ToWin32Color(line.color));
            brushes.emplace(line.color, hpen);
        }

        SelectObject(hdc, hpen);

        // Draw initial point, then LineTo the rest
        MoveToEx(hdc, line.p1.first, rect.bottom - line.p1.second, NULL);
        LineTo(hdc, line.p2.first, rect.bottom - line.p2.second);
    }

    // Draw Text
    for (GuiText text : m_windowState.text)
    {
        DoDrawText(hdc, text, rect);
    }

    // Delete HPEN objects
    for (const auto& pair : brushes)
    {
        if (pair.second)
            DeleteObject(pair.second);
    }

    EndPaint(m_hwnd, &ps);
}

void Plot2D::Win32Window::InvalidateRegion(const Plot2D::WindowRect& windowRect,
                                           const Plot2D::WindowState& windowState)
{
    m_windowState = windowState;
    RECT win32Rect;
    GetClientRect(m_hwnd, &win32Rect);

    // Universal coords assume origin at bottom left and extend up. Win32 is the opposite
    const RECT rect = {windowRect.left, win32Rect.bottom - windowRect.top, windowRect.right,
                       win32Rect.bottom - windowRect.bottom};
    InvalidateRect(m_hwnd, &rect, FALSE);
}

void Plot2D::Win32Window::Invalidate(const Plot2D::WindowState& windowState)
{
    m_windowState = windowState;
    InvalidateRect(m_hwnd, nullptr, FALSE);
}

#elif defined(__linux__)

#elif defined(__APPLE__)

#endif

std::unique_ptr<Plot2D::IGraphicsContext> Plot2D::m_graphicsContext = nullptr;

template <typename T>
Plot2D::Plot2D(const std::vector<T>& x, const std::vector<T>& y, const std::string& title,
               const std::string& xLabel, const std::string& yLabel)
    : xLabel(xLabel), yLabel(yLabel), title(title)
{
    // Check that x and y vectors are the same size and are numeric
    static_assert(std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
                  "Plot2D requires a numeric type for T (bool is not allowed)");

    //assert(!x.empty() && !y.empty(), "");
    assert(x.size() == y.size());

    datasetSize = x.size();

    for (size_t i = 0; i < datasetSize; ++i)
    {
        m_data.emplace_back(static_cast<float>(x[i]), static_cast<float>(y[i]));
    }

    
    // Get largest and smallest points in dataset
    std::pair<float, float> p;
    float largestX = -(std::numeric_limits<float>::max)(),
          largestY = -(std::numeric_limits<float>::max)();
    float smallestX = (std::numeric_limits<float>::max)(),
          smallestY = (std::numeric_limits<float>::max)();
    for (int i = 0; i < datasetSize; i++)
    {
        p = m_data[i];
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

    // The data span is the differnce between the largest and smallest points
    dataSpanX = largestX - smallestX;
    dataSpanY = largestY - smallestY;

    // The conversion from data-space to plot-space requires both coord systems to start from (0,
    // 0). These values will be used to adjust the m_data points during transformation to achieve
    // this.
    PlotZeroOffset.first = smallestX;
    PlotZeroOffset.second = smallestY;

#ifdef _WIN32
    m_graphicsContext = std::make_unique<Win32GraphicsContext>();
#elif defined(__APPLE__)
#ifdef __OBJC__
    m_graphicsContext = CocoaGraphicsContext();
#endif
#elif defined(__linux__)
    m_graphicsContext = X11GraphicsContext();
#else
    std::throw(std::exception("Unsupported platform."));
#endif
    m_graphicsContext->Init();
    m_window =
        m_graphicsContext->MakeWindow(Color::Black(), defaultWindowSize, minWindowSize, false, title);
    m_window->OnMouseHoverCallback = [this](Point p) { this->OnMouseHoverCallback(*m_window, p); };
    m_window->OnResizeCallback = [this]() { this->OnWindowResizeCallback(*m_window); };
    m_window->OnResizeEndCallback = [this]() { this->OnWindowResizeEndCallback(*m_window); };
    m_window->AddMenuButton("File", "Save", [this]() { this->OnSaveButtonClicked(*m_window); });

    UpdatePlotWindowState(m_plotWindowState, *m_window);

   /* m_window->Invalidate(m_plotWindowState);
    m_window->SetIsVisible(true);*/
}

void Plot2D::UpdatePlotWindowState(Plot2D::WindowState& windowState, IWindow& window)
{
    windowState.background = Color::Black();
    windowState.polylines.clear();
    windowState.polylines.push_back(GetDataPolyline(
        m_data, Color::Green(),
        window));  // TODO: Make sure different plot types (scatter, bar, etc...) are supported
    windowState.polylines.push_back(GetPlotBorderPolyline(window, Color::White()));

    std::pair<std::vector<GuiLine>, std::vector<GuiText>> ticks =
        GetPlotBorderTickLines(window, Color::White());
    windowState.lines.clear();
    windowState.text.clear();
    windowState.lines.insert(windowState.lines.end(), ticks.first.begin(), ticks.first.end());
    windowState.text.insert(windowState.text.end(), ticks.second.begin(), ticks.second.end());

    std::vector<GuiText> labels = GetPlotLabels(window, Color::White());
    windowState.text.insert(windowState.text.end(), labels.begin(), labels.end());
}

Plot2D::GuiPolyline Plot2D::GetPlotBorderPolyline(IWindow& window, Color color)
{
    WindowRect rect = window.GetRect();
    const int leftBorderPos = static_cast<int>(rect.left + m_plotBorderOffsets.first);
    const int rightBorderPos = static_cast<int>(rect.right - m_plotBorderOffsets.first);
    const int topBorderPos = static_cast<int>(rect.top - m_plotBorderOffsets.second);
    const int bottomBorderPos = static_cast<int>(rect.bottom + m_plotBorderOffsets.second);

    // Bottom left corner -> top left corner -> top right corner ->
    // Bottom right corner -> bottom left corner
    std::vector<Point> borderPoints{{leftBorderPos, bottomBorderPos},
                                    {leftBorderPos, topBorderPos},
                                    {rightBorderPos, topBorderPos},
                                    {rightBorderPos, bottomBorderPos},
                                    {leftBorderPos, bottomBorderPos}};

    return GuiPolyline(borderPoints, color, 1);
}

std::pair<std::vector<Plot2D::GuiLine>, std::vector<Plot2D::GuiText>>
Plot2D::GetPlotBorderTickLines(IWindow& window, Color color)
{
    // TODO: Consolidate magic numbers

    WindowRect rect = window.GetRect();
    const int leftBorderPos = static_cast<int>(rect.left + m_plotBorderOffsets.first);
    const int rightBorderPos = static_cast<int>(rect.right - m_plotBorderOffsets.first);
    const int topBorderPos = static_cast<int>(rect.top - m_plotBorderOffsets.second);
    const int bottomBorderPos = static_cast<int>(rect.bottom + m_plotBorderOffsets.second);

    std::vector<GuiLine> ticks;
    std::vector<GuiText> labels;
    std::stringstream label;

    // Draw X-axis ticks
    int numTicksX = 4;
    int x = 0;
    float offset = PlotZeroOffset.first;
    int tickInterval = (rightBorderPos - leftBorderPos) / (numTicksX + 1);
    float increment = dataSpanX / (numTicksX + 1);
    for (int i = 1; i <= numTicksX; ++i)
    {
        x = leftBorderPos + i * tickInterval;
        offset += increment;

        // Add tick line
        ticks.push_back(
            GuiLine({x, bottomBorderPos + tickLength}, {x, bottomBorderPos - tickLength}, color));

        // Add tick label
        label.str("");
        label << std::setprecision(4) << offset;
        labels.push_back(GuiText(label.str(), {x - 10, max(bottomBorderPos - 10, rect.bottom + 20)},
                                 1, color, Orientation::HORIZONTAL));
    }
    ticks.push_back(GuiLine({rightBorderPos, bottomBorderPos + tickLength},
                            {rightBorderPos, bottomBorderPos - tickLength}, color));
    label.str("");
    label << std::setprecision(4) << offset + increment;
    labels.push_back(GuiText(label.str(),
                             {rightBorderPos - 10, max(bottomBorderPos - 10, rect.bottom + 20)}, 1,
                             color, Orientation::HORIZONTAL));

    // Draw Y-axis ticks
    int numTicksY = 4;
    int y = 0;
    increment = dataSpanY / (numTicksY + 1);
    offset = PlotZeroOffset.second;
    tickInterval = (topBorderPos - bottomBorderPos) / (numTicksY + 1);
    int avgCharWidth = window.GetAverageCharWidth();
    for (int i = 1; i <= numTicksY; i++)
    {
        y = bottomBorderPos + (i * tickInterval);
        offset += increment;

        // Add tick line
        ticks.push_back(
            GuiLine({leftBorderPos - tickLength, y}, {leftBorderPos + tickLength, y}, color));

        // Add tick label
        label.str("");
        label << std::setprecision(4) << offset;
        labels.push_back(GuiText(
            label.str(),
            {(int)max(10, leftBorderPos - avgCharWidth * (label.str().size() + 1) - tickLength), y - 4},
            1, color, Orientation::HORIZONTAL));
    }
    ticks.push_back(GuiLine({leftBorderPos - tickLength, topBorderPos},
                            {leftBorderPos + tickLength, topBorderPos}, color));
    label.str("");
    label << std::setprecision(4) << offset;
    labels.push_back(GuiText(
        label.str(),
        {(int)max(10, leftBorderPos - avgCharWidth * (label.str().size() + 1) - tickLength), topBorderPos - 4}, 1,
        color, Orientation::HORIZONTAL));

    return {ticks, labels};
}

std::vector<Plot2D::GuiText> Plot2D::GetPlotLabels(IWindow& window, Color color)
{
    WindowRect rect = window.GetRect();

    std::vector<GuiText> labels{3};
    labels[0] = GuiText(xLabel, Point(rect.right / 2, rect.bottom + 20), 1, color,
                        Orientation::HORIZONTAL);  // X-axis label
    labels[1] = GuiText(yLabel, Point(rect.left + 20, rect.top / 2), 1, color,
                        Orientation::VERTICAL);  // Y-axis label
    labels[2] = GuiText(title, Point(rect.right / 2, rect.top - 10), 1, color,
                        Orientation::HORIZONTAL);  // Plot title

    return labels;
}

Plot2D::GuiPolyline Plot2D::GetDataPolyline(const std::vector<std::pair<float, float>>& data,
                                            Color color, IWindow& window)
{
    WindowRect rect = window.GetRect();
    const float xPlotSpan = rect.right - 2 * m_plotBorderOffsets.first;
    const float yPlotSpan = rect.top - 2 * m_plotBorderOffsets.second;
    const float xScale = xPlotSpan / dataSpanX;
    const float yScale = yPlotSpan / dataSpanY;

    std::vector<Point> lines(datasetSize);  // Allocate enough spaces for dataset
    std::vector<uint8_t> pixel_mask(rect.top * rect.right, 0);
    std::vector<Point> to_draw;
    to_draw.reserve(lines.size());

    size_t idx;
    int x, y;
    Point transformedPoint;
    std::pair<float, float> point;
    for (int i = 0; i < datasetSize; i++)
    {
        point = data[i];
        transformedPoint = {static_cast<int>(((point.first - PlotZeroOffset.first) * xScale) +
                                             m_plotBorderOffsets.first),
                            static_cast<int>(((point.second - PlotZeroOffset.second) * yScale) +
                                             m_plotBorderOffsets.second)};
        idx = transformedPoint.second * rect.right + transformedPoint.first;
        if (!pixel_mask[idx])
        {
            pixel_mask[idx] = 1;
            to_draw.push_back(transformedPoint);
        }
        //lines[i] = Point(x, y);
    }

    GuiPolyline polyline(to_draw, color);
    return polyline;
}

std::pair<float, float> Plot2D::AbsToPlot(IWindow& window, const int x, const int y,
                                          const std::pair<float, float>& plotBorderOffset)
{
    WindowRect rect = window.GetRect();
    if ((x >= plotBorderOffset.first && x <= rect.right - plotBorderOffset.first) &&
        (y >= plotBorderOffset.second && y <= rect.bottom - plotBorderOffset.second))
    {
        // Scale absolute mouse coordinates to plot window space
        const float xPlotSpan = (rect.right - rect.left) - 2 * plotBorderOffset.first;
        const float yPlotSpan = (rect.top - rect.bottom) - 2 * plotBorderOffset.second;
        float xScale = dataSpanX / xPlotSpan;
        float yScale = dataSpanY / yPlotSpan;

        // In win32 windows, (0,0) is upper left corner, so Y needs to be reversed
        return std::pair<float, float>(
            (x - plotBorderOffset.first) * xScale + PlotZeroOffset.first,
            ((yPlotSpan - y + plotBorderOffset.second) * yScale + PlotZeroOffset.second));
    }
    else
    {
        // Outside the plot window just default to (0,0)
        return std::pair<float, float>(0.0f, 0.0f);
    }
}
void Plot2D::Show(bool block)
{
    m_window->SetIsVisible(true);
    if (block)
        m_window->RunEventLoop();
}
inline std::pair<float, float> Plot2D::GetPlotBorderOffsets()
{
    return m_plotBorderOffsets;
}
void Plot2D::SetPlotBorderOffsets(std::pair<float, float> offsets)
{
    m_plotBorderOffsets = offsets;
}

void Plot2D::OnWindowResizeCallback(IWindow& window)
{
    std::pair<int, int> size = window.GetRect().Size();

    SetPlotBorderOffsets(std::pair<float, float>{size.first * m_plotBorderOffsetFactor,
                                                 size.second * m_plotBorderOffsetFactor});

    if (datasetSize < m_deferredResizeLimit)
    {
        UpdatePlotWindowState(m_plotWindowState, *m_window);
        window.Invalidate(m_plotWindowState);
    }
}
void Plot2D::OnWindowResizeEndCallback(IWindow& window)
{
    std::pair<int, int> size = window.GetRect().Size();

    SetPlotBorderOffsets(std::pair<float, float>{size.first * m_plotBorderOffsetFactor,
                                                 size.second * m_plotBorderOffsetFactor});
    UpdatePlotWindowState(m_plotWindowState, *m_window);
    window.Invalidate(m_plotWindowState);
}

// TODO: Refactor
std::pair<float, float> Plot2D::GetTransformedCoordinates(const int& x, const int& y,
                                                                         const int windowWidth,
                                                                         const int& windowHeight)
{
    // Scale absolute coordinates to plot window space
    const float xPlotSpan = windowWidth - 2 * m_plotBorderOffsets.first;
    const float yPlotSpan = windowHeight - 2 * m_plotBorderOffsets.second;
    float xScale = dataSpanX / xPlotSpan;
    float yScale = dataSpanY / yPlotSpan;

    return std::pair<float, float>(
        (x - m_plotBorderOffsets.first) * xScale + PlotZeroOffset.first,
        ((y - m_plotBorderOffsets.second) * yScale + PlotZeroOffset.second));
}
void Plot2D::OnMouseHoverCallback(IWindow& w, Point mousePos)
{
    std::pair<float, float> plotBorderOffsets = GetPlotBorderOffsets();
    WindowRect rect = w.GetRect();
    bool inside = (mousePos.first >= plotBorderOffsets.first &&
                   mousePos.first <= (rect.Size().first) - plotBorderOffsets.first) &&
                  (mousePos.second >= plotBorderOffsets.second &&
                   mousePos.second <= (rect.Size().second) - plotBorderOffsets.second);

    auto transformedCoords = GetTransformedCoordinates(mousePos.first, mousePos.second,
                                                       rect.Size().first, rect.Size().second);
    std::string coordText = "X: " + std::to_string(transformedCoords.first) +
                            " Y: " + std::to_string(transformedCoords.second);

    // Start mouse coordinate text at top left & 1 unit from top
    int x = rect.left;
    int y = rect.top - 1;

    // mouse coordinates should fit inside a rect that starts at the top left corner and extends 200
    // wide and 30 high
    WindowRect mouseCoordinateRect(rect.top, rect.left, rect.left + 200,
                                   rect.top - 30);  // TODO: validate
    if (inside)
    {
        WindowState mouseCoordinates;
        mouseCoordinates.text = {
            GuiText(coordText, Point(x, y), 1, Color::White(), Orientation::HORIZONTAL)};
        mouseCoordinates.background = Color::Black();
        w.InvalidateRegion(mouseCoordinateRect, mouseCoordinates);
    }
    else
    {
        WindowState empty;
        empty.background = Color::Black();
        w.InvalidateRegion(mouseCoordinateRect, empty);
    }
}

void Plot2D::OnSaveButtonClicked(IWindow& w)
{
    w.SaveScreenshotAsPNG(title + "_" + w.GetTimestamp());
}

}  // namespace cpplot2d