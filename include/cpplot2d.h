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

// Enable by defining CPLOT2D_ENABLE_DEBUG (in CMake or project preprocessor defs)
#define CPPLOT2D_ENABLE_DEBUG


#ifdef _WIN32
#include <windows.h>
#endif

// Enable by defining CPLOT2D_ENABLE_DEBUG (in CMake or project preprocessor defs)
#ifdef CPPLOT2D_ENABLE_DEBUG

// Print to stderr and, on Windows, send to the debugger output.
#define CPPLOT2D_DEBUG(fmt, ...)                                                \
    do {                                                                       \
        std::fprintf(stderr, "[cpplot2d DEBUG] %s:%d:%s(): " fmt "\n",         \
                     __FILE__, __LINE__, __func__, ##__VA_ARGS__);            \
        std::fflush(stderr);                                                   \
        /* also send to Visual Studio Output window */                         \
        /* small, non-portable helper */                                       \
        /* Note: OutputDebugString expects a wide string on Unicode builds; */ \
        /* using ANSI here for simplicity (safe in most dev scenarios). */     \
        /* If you need wide strings, convert fmt+args into a std::string. */  \
        /* Keep this call optional to avoid depending on windows.h globally. */\
        do {                                                                   \
        } while (0);                                                           \
    } while (0)

#else

// No-op in release builds
#define CPPLOT2D_DEBUG(fmt, ...) ((void)0)

#endif

#ifdef _WIN32
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

// Color struct representing RGBA color values.
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

    /**
     Creates a Color from standard RGBA values.
     @param red Red component (0-255)
     @param green Green component (0-255)
     @param blue Blue component (0-255)
     @param alpha Alpha component (0-255), default is 255 (opaque)

    */
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

// Plot appearance properties
struct PlotProperties
{
   public:
    Color backgroundColor = Color::Black();
    Color borderColor = Color::White();
    bool showLegend = true;
    const int tickLineCount = 10;
    int labelTextSpacing = 5;
    const std::pair<int, int> minWindowSize = {200, 100};
    const std::pair<int, int> defaultWindowSize = {800, 600};
};

class Plot2D
{
   public:
    Plot2D(const std::string& title = "Plot", const std::string& xLabel = "x",
           const std::string& yLabel = "y", PlotProperties props = {});

    /**
     Adds a line series to the plot.
     @tparam T Numeric type of the input data (e.g., float, double, int)
     @param x Vector of x-coordinates
     @param y Vector of y-coordinates
     @param color Color of the line (default: green)
     @param size Thickness of the line (default: 1)
    * */
    template <typename T>
    Plot2D& AddLine(const std::vector<T>& x, const std::vector<T>& y, Color color = Color::Green(), int size = 1);


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
     Sets the bottom limit for deferring window redraws until the window is idle. Decrease this
     value if resizing the window causes flickering.

     @param limit the number of plot points beyond which the window will deferr redrawing until it
     is in an idle state.
     */
    void SetDeferredDrawLimit(int limit = 5000);

   protected:
    using Point = std::pair<int, int>;

    // Text orientation
    enum class Orientation : uint8_t
    {
        HORIZONTAL = 0,
        VERTICAL = 1
    };

    // Lightweight view into an array. C++17 implementation of span.
    template <typename T>
    class ArrayView
    {
       public:
        ArrayView() : m_data(nullptr), m_size(0)
        {
        }
        ArrayView(const T* data, size_t size) : m_data(data), m_size(size)
        {
        }
        template <typename Alloc>
        ArrayView(const std::vector<T, Alloc>& v) : m_data(v.data()), m_size(v.size())
        {
        }

        const T* data() const
        {
            return m_data;
        }
        size_t size() const
        {
            return m_size;
        }
        const T& operator[](size_t i) const
        {
            return m_data[i];
        }

       private:
        const T* m_data;
        size_t m_size;
    };

    // Data series
    class Series
    {
       public:
        Series(ArrayView<float> xs, ArrayView<float> ys) : x(xs), y(ys)
        {
        }
        ArrayView<float> x;
        ArrayView<float> y;
    };

    // Series for line plots
    class LineSeries : public Series
    {
       public:
        LineSeries(ArrayView<float> xs, ArrayView<float> ys, Color color, int size = 1)
            : Series(xs, ys), size(size), color(color)
        {
        }
        int size;
        Color color;
    };

    // Series for scatter plots
    class ScatterSeries : public Series
    {
       public:
        ScatterSeries(ArrayView<float> xs, ArrayView<float> ys, Color color, int pointSize = 1)
            : Series(xs, ys), pointSize(pointSize), color(color)
        {
        }
        int pointSize;
        Color color;
    };
    struct ScrollEvent
    {
       public:
        double deltaX;  // positive = right
        double deltaY;  // positive = up
        bool precise;   // smooth scroll (true = from touchpad, false = wheel)
        double scale;   // pixels per unit
    };

    /*
     *    (left, top)
     *     +-----------------+
     *     |                 |
     *     |                 |
     *     |                 |
     *     +-----------------+
     *                 (right, bottom)
     *
     *  Notes:
     *   - Constructor: WindowRect(top, left, right, bottom)
     *   - Size() returns { width = right - left, height = top - bottom }
     *   - This codebase uses a universal coordinate system where origin is at the
     *     bottom-left for drawing logic
    */
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

        int top = 0;
        int left = 0;
        int right = 0;
        int bottom = 0;
    };

    // Helper for generating unique IDs
    class IDGenerator
    {
       public:
        static int Next()
        {
            static int current = 1000;
            return current++;
        }
    };

    // Helper for creating file names
    class FileName
    {
       public:
        static std::string Create(const std::string& dir, const std::string& filename);
    };

    // Represents a single point to be drawn in the window
    struct GuiPoint
    {
        Point p;
        int size;
    };

    // Represents a polyline to be drawn in the window. More efficient than multiple lines.
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

    // Represents a single line to be drawn in the window
    struct GuiLine
    {
       public:
        GuiLine(Point p1, Point p2, Color c) : p1(p1), p2(p2), color(c)
        {
        }
        Point p1, p2;
        Color color;
    };

    // Represents a text entry to be drawn in the window
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

    // Represents a rectangle to be drawn in the window
    struct GuiRect
    {
        Point topLeft;
        Point bottomRight;
        Color fillColor;
        Color borderColor;
        int borderWidth;
    };

    // Represents a circle to be drawn in the window
    struct GuiCircle
    {
        Point center;
        int radius;
        Color fillColor;
        Color borderColor;
        int borderWidth;
    };

    // Represents the entire state of the window to be drawn
    struct WindowState
    {
        std::vector<GuiText> text;
        std::vector<GuiLine> lines;           
        std::vector<GuiPolyline> polylines; 
        std::vector<GuiPoint> points;       
        std::vector<GuiRect> rects;       
        std::vector<GuiCircle> circles;     
        Color background;
        std::pair<int, int> minSize;
    };

    // Interface for platform-specific window implementations
    class IWindow
    {
       public:

        // Draws text at given x and y position in window space. Returns id to that text
        virtual void DrawTextAt(const std::string text, int spacing, int size = 1,
                                Point startPos = {0, 0},
                                Orientation orientation = Orientation::HORIZONTAL,
                                Color color = Color::White()) = 0;

        // Invalidates entire window, forcing a redraw
        virtual void Invalidate(std::shared_ptr<WindowState> windowState) = 0;
        virtual int GetAverageCharWidth() = 0;
        virtual void DrawWindowState() = 0;

        // Invalidates only the space  encompassed in the given rect
        virtual void InvalidateRegion(const WindowRect& rect, const WindowState& windowState) = 0;
        virtual void AddMenuButton(const std::string menu, const std::string label,
                                   std::function<void()> onClickCallback) = 0;
        virtual bool SaveScreenshotAsPNG(const std::string& fileName) = 0;
        virtual void SetIsVisible(bool isVisible) = 0;

        // Returns a timestamp string for use in file names in the format YYYYMMDD_HHMMSS
        virtual std::string GetTimestamp() = 0;
        virtual WindowRect GetRect() = 0;
        virtual void RunEventLoop() = 0;
        std::function<void(Point)> OnMouseHoverCallback;
        std::function<void()> OnResizeStartCallback;
        std::function<void()> OnResizeEndCallback;
        std::function<void()> OnResizeCallback;
        std::function<void(ScrollEvent)> OnScrollWheelMoveCallback;
    };

    // Interface for platform-specific graphics context implementations
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
    };

    static std::unique_ptr<IGraphicsContext> m_graphicsContext;
    std::unique_ptr<IWindow> m_window;
    void UpdateOffsets(const std::vector<float>& x, const std::vector<float>& y);
    void OnMouseHoverCallback(IWindow& window, Point mousePos);
    void OnWindowResizeCallback(IWindow& window);
    void OnSaveButtonClicked(IWindow& window);
    void OnResetZoomButtonClicked(IWindow& window);
    void OnWindowResizeEndCallback(IWindow& window);
    void OnScrollWheelMoveCallback(ScrollEvent wheelEvent, IWindow& window);
    std::pair<float, float> GetPlotBorderOffsets();
    void SetViewportRect(const WindowRect& rect);
    void SetPlotBorderOffsets(std::pair<float, float> offsets);
    std::pair<float, float> GetViewportToWindowScaleFactors(const int windowWidth, const int windowHeight);
    std::pair<float, float> GetWindowToViewportScaleFactors(const int windowWidth, const int windowHeight);
    std::pair<float, float> GetTransformedCoordinates(const int& x, const int& y,
                                                              const int windowWidth,
                                                              const int& windowHeight);
    void UpdatePlotWindowState(WindowState* windowState, IWindow& window);
    // Returns data points as polyline based on window sizes
    // TOOD: account for zoom here?
    GuiPolyline GetDataPolyline(const LineSeries& series, IWindow& window);

    // Returns plot border as polyline
    GuiPolyline GetPlotBorderPolyline(IWindow& window, Color color);
    std::pair<std::vector<GuiLine>, std::vector<GuiText>> GetPlotBorderTickLines(IWindow& window,
                                                                                 Color color);
    std::vector<GuiText> GetPlotLabels(IWindow& window, Color color);

    struct m_dataSeries
    {
        std::vector<LineSeries> lines;
        std::vector<ScatterSeries> points;
    } m_dataSeries;
    
    std::pair<float, float> PlotZeroOffset = {(std::numeric_limits<float>::max)(),
                                              (std::numeric_limits<float>::max)()};
    std::pair<float, float> m_plotBorderOffsets;    // TODO: REMOVE
    WindowRect m_viewportRect = {0,0,0,0};               // current viewport in window space

    // runtime view (what is currently shown). Defaults initialized when data is first added.
    std::pair<float, float> m_viewZero = {0.0f, 0.0f};  // data-space lower-left of current view
    float m_viewSpanX = 0.0f;                           // data-space width of current view
    float m_viewSpanY = 0.0f;                           // data-space height of current view

    // Whether to invalidate small region for mouse coords. 
    // Useful to prevent invalidating more than once.
    bool m_invalidateMouseCoordRegion = false;  

    std::pair<float, float> m_defaultViewZero = {0.0f, 0.0f};
    float m_defaultViewSpanX = 0.0f;
    float m_defaultViewSpanY = 0.0f;
    const float m_plotBorderOffsetFactor = 0.125;
    std::string xLabel;
    std::string yLabel;
    std::string title;
    float dataSpanX = 0;
    float dataSpanY = 0;
    float largestX = -(std::numeric_limits<float>::max)(),
          largestY = -(std::numeric_limits<float>::max)();
    float smallestX = (std::numeric_limits<float>::max)(),
          smallestY = (std::numeric_limits<float>::max)();
    const int tickLength = 4;
    const std::pair<int, int> minWindowSize = {200, 100};
    std::pair<int, int> defaultWindowSize = {800, 600};
    PlotProperties m_plotProperties;
    std::shared_ptr<WindowState> m_plotWindowState = std::make_shared<WindowState>();

#ifdef _WIN32  // Windows-specific implementation
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
       protected:
        ULONG_PTR m_gdiplusToken;
    };

    class Win32Window : public Plot2D::IWindow
    {
       public:
        Win32Window(std::pair<int, int> defaultWindowSize, std::pair<int, int> pos,
                    std::string title);

        void DrawTextAt(const std::string text, int spacing, int size = 1, Point startPos = {0,0},
                        Orientation orientation = Orientation::HORIZONTAL, Color color = Color::White()) override;
        int GetAverageCharWidth() override;

        // TODO: This is currently designed to treat each new menu button as a new menu.
        // Modify to allow multiple buttons under same menu?
        void AddMenuButton(const std::string menu, const std::string label,
                           std::function<void()> onClickCallback) override;
        void SetIsVisible(bool isVisible) override;
        void Invalidate(std::shared_ptr<WindowState> windowState) override;
        void DrawWindowState() override;
        std::string GetTimestamp() override;
        void RunEventLoop() override;
        void InvalidateRegion(const WindowRect& rect, const WindowState& windowState) override;
        bool BrowseForFolder(std::string& outFolder);
        WindowRect GetRect() override;
        bool SaveScreenshotAsPNG(const std::string& fileName) override;
        static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

       protected:
        std::shared_ptr<WindowState> m_windowState;
        UINT m_wheelScrollLines;
        HWND m_hwnd;
        HMENU m_hMenu;
        std::map<int, std::function<void()>> m_menuCommands;

        LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
        void SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename);
        HBITMAP CaptureWindowContent(HWND hwnd);
        COLORREF ToWin32Color(const Color color);
        void DoDrawText(HDC hdc, GuiText text, RECT clientRect);
    };

#elif defined(__linux__) 
    // Linux-specific implementation would go here

#elif defined(__APPLE__)
    // macOS-specific implementation would go here

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
    
    SystemParametersInfo(SPI_GETWHEELSCROLLLINES, 0, &m_wheelScrollLines, 0);
    if (!m_hwnd)
    {
        DWORD err = GetLastError();
        std::cout << "Failed to create window. Error: " << err << std::endl;
    }
    m_hMenu = CreateMenu();
}

void Plot2D::Win32Window::AddMenuButton(const std::string menu, const std::string label,
                                        std::function<void()> onClickCallback)
{
    HMENU hNewMenu = CreateMenu();
    int id = IDGenerator::Next();
    AppendMenu(hNewMenu, MF_STRING, id, label.c_str());
    AppendMenu(m_hMenu, MF_POPUP, (UINT_PTR)hNewMenu, menu.c_str());

    SetMenu(m_hwnd, m_hMenu);

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
        case WM_MOUSEWHEEL:
        {
            int delta = GET_WHEEL_DELTA_WPARAM(wParam);  // +/- 120 per notch
            double pixelDelta = delta / 120.0 * m_wheelScrollLines * 16.0;  // rough conversion

            ScrollEvent e;
            e.deltaX = 0;
            e.deltaY = -pixelDelta;
            e.precise = false;
            e.scale = 1.0;

            if (OnScrollWheelMoveCallback) OnScrollWheelMoveCallback(e);

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
            std::pair<int, int> sizes = m_windowState->minSize;
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
    m_windowState->text.push_back(GuiText(text, startPos, spacing, color, orientation));
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
    HBRUSH blackBrush = CreateSolidBrush(ToWin32Color(m_windowState->background));
    FillRect(hdc, &rect, blackBrush);
    DeleteObject(blackBrush);

    // Draw Polylines
    std::map<Color, HPEN> brushes;  // HPEN hashmap
    HPEN hpen = nullptr;
    std::map<Color, HPEN>::iterator it;
    for (GuiPolyline polyline : m_windowState->polylines)
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
        if (size <= 0) continue;

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
    for (GuiLine line : m_windowState->lines)
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
    for (GuiText text : m_windowState->text)
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
    m_windowState = std::make_shared<WindowState>(windowState);
    RECT win32Rect;
    GetClientRect(m_hwnd, &win32Rect);

    // Universal coords assume origin at bottom left and extend up. Win32 is the opposite
    const RECT rect = {windowRect.left, win32Rect.bottom - windowRect.top, windowRect.right,
                       win32Rect.bottom - windowRect.bottom};
    CPPLOT2D_DEBUG("Invalidating Region %i %i %i %i", rect.top, rect.left, rect.bottom, rect.right);
    InvalidateRect(m_hwnd, &rect, FALSE);
}

void Plot2D::Win32Window::Invalidate(std::shared_ptr<Plot2D::WindowState> windowState)
{
    m_windowState = windowState;
    CPPLOT2D_DEBUG("Invalidating");

    InvalidateRect(m_hwnd, nullptr, FALSE);
}

#elif defined(__linux__)

#elif defined(__APPLE__)

#endif

std::unique_ptr<Plot2D::IGraphicsContext> Plot2D::m_graphicsContext = nullptr;

Plot2D::Plot2D(const std::string& title, const std::string& xLabel, const std::string& yLabel, PlotProperties props)
    : xLabel(xLabel), yLabel(yLabel), title(title), m_plotProperties(props)
{

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
    m_window->OnScrollWheelMoveCallback = [this](ScrollEvent e) { this->OnScrollWheelMoveCallback(e, *m_window); };
    m_window->AddMenuButton("File", "Save", [this]() { this->OnSaveButtonClicked(*m_window); });
    m_window->AddMenuButton("View", "Reset Zoom", [this]() { this->OnResetZoomButtonClicked(*m_window); });

    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
}

void Plot2D::UpdateOffsets(const std::vector<float>& x, const std::vector<float>& y)
{
    // Get largest and smallest points in dataset
    std::pair<float, float> p;
    int size = static_cast<int>(x.size());
    for (int i = 0; i < size; i++)
    {
        p = {x[i], y[i]};
        if (p.first > largestX) largestX = p.first;
        if (p.first < smallestX) smallestX = p.first;
        if (p.second > largestY) largestY = p.second;
        if (p.second < smallestY) smallestY = p.second;
    }

    // The data span is the differnce between the largest and smallest points
    dataSpanX = max((largestX - smallestX), dataSpanX);
    dataSpanY = max((largestY - smallestY), dataSpanY);

    // The conversion from data-space to plot-space requires both coord systems to start from (0,
    // 0). These values will be used to adjust the m_data points during transformation to achieve
    // this.
    PlotZeroOffset.first = min(PlotZeroOffset.first, smallestX);
    PlotZeroOffset.second = min(PlotZeroOffset.second, smallestY);

    m_viewZero = PlotZeroOffset;
    m_viewSpanX = dataSpanX > 0.0f ? dataSpanX : 1.0f;
    m_viewSpanY = dataSpanY > 0.0f ? dataSpanY : 1.0f;

    m_defaultViewZero = m_viewZero;
    m_defaultViewSpanX = m_viewSpanX;
    m_defaultViewSpanY = m_viewSpanY;
 
}

template <typename T>
Plot2D& Plot2D::AddLine(const std::vector<T>& x, const std::vector<T>& y, Color color, int size)
{
    // Check that x and y vectors are the same size and are numeric
    static_assert(std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
                  "Plot2D requires a numeric type for T (bool is not allowed)");
    assert(x.size() == y.size());

    m_dataSeries.lines.emplace_back(x, y, color, size);
    UpdateOffsets(x, y);  // Update plot offsets based on new data to make sure plot fits all data

    return *this;
}

void Plot2D::UpdatePlotWindowState(Plot2D::WindowState* windowState, IWindow& window)
{
    windowState->background = Color::Black();
    windowState->polylines.clear();
    for (const LineSeries& series : m_dataSeries.lines)
    {
        windowState->polylines.push_back(GetDataPolyline(
            series,
            window));
    }
   
    windowState->polylines.push_back(GetPlotBorderPolyline(window, Color::White()));

    std::pair<std::vector<GuiLine>, std::vector<GuiText>> ticks =
        GetPlotBorderTickLines(window, Color::White());
    windowState->lines.clear();
    windowState->text.clear();
    windowState->lines.insert(windowState->lines.end(), ticks.first.begin(), ticks.first.end());
    windowState->text.insert(windowState->text.end(), ticks.second.begin(), ticks.second.end());

    std::vector<GuiText> labels = GetPlotLabels(window, Color::White());
    windowState->text.insert(windowState->text.end(), labels.begin(), labels.end());
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
    float offset = m_viewZero.first;
    int tickInterval = (rightBorderPos - leftBorderPos) / (numTicksX + 1);
    float increment = m_viewSpanY / (numTicksX + 1);
    for (int i = 1; i <= numTicksX; ++i)
    {
        x = leftBorderPos + i * tickInterval;
        offset += increment;

        // Add tick line
        ticks.push_back(
            GuiLine({x, bottomBorderPos + tickLength}, {x, bottomBorderPos - tickLength}, color));

        // Add tick label
        label.str("");
        label << std::setprecision(3) << offset;
        labels.push_back(GuiText(label.str(), {x - 10, max(bottomBorderPos - 10, rect.bottom + 20)},
                                 1, color, Orientation::HORIZONTAL));
    }
    ticks.push_back(GuiLine({rightBorderPos, bottomBorderPos + tickLength},
                            {rightBorderPos, bottomBorderPos - tickLength}, color));
    label.str("");
    label << std::setprecision(3) << offset + increment;
    labels.push_back(GuiText(label.str(),
                             {rightBorderPos - 10, max(bottomBorderPos - 10, rect.bottom + 20)}, 1,
                             color, Orientation::HORIZONTAL));

    // Draw Y-axis ticks
    int numTicksY = 4;
    int y = 0;
    increment = m_viewSpanY / (numTicksY + 1);
    offset = m_viewZero.second;
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
        label << std::setprecision(3) << offset;
        labels.push_back(GuiText(
            label.str(),
            {(int)max(10, leftBorderPos - avgCharWidth * (label.str().size() + 1) - tickLength), y - 2},
            1, color, Orientation::HORIZONTAL));
    }
    ticks.push_back(GuiLine({leftBorderPos - tickLength, topBorderPos},
                            {leftBorderPos + tickLength, topBorderPos}, color));
    label.str("");
    label << std::setprecision(3) << offset;
    labels.push_back(GuiText(
        label.str(),
        {(int)max(10, leftBorderPos - avgCharWidth * (label.str().size() + 1) - tickLength), topBorderPos - 2}, 1,
        color, Orientation::HORIZONTAL));

    return {ticks, labels};
}

std::vector<Plot2D::GuiText> Plot2D::GetPlotLabels(IWindow& window, Color color)
{
    WindowRect rect = window.GetRect();

    std::vector<GuiText> labels{3};
    labels[0] = GuiText(xLabel, Point(rect.right / 2, rect.bottom + 20), 1, color,
                        Orientation::HORIZONTAL);  // X-axis label
    labels[1] = GuiText(yLabel, Point(rect.left + 10, rect.top / 2), 1, color,
                        Orientation::VERTICAL);  // Y-axis label
    labels[2] = GuiText(title, Point(rect.right / 2, rect.top - 10), 1, color,
                        Orientation::HORIZONTAL);  // Plot title

    return labels;
}
inline std::pair<float, float> Plot2D::GetViewportToWindowScaleFactors(const int windowWidth, const int windowHeight)
{
    const float xPlotSpan = windowWidth - 2 * m_plotBorderOffsets.first;
    const float yPlotSpan = windowHeight - 2 * m_plotBorderOffsets.second;

    const float viewSpanX = (m_viewSpanX > 0.0f) ? m_viewSpanX : 1.0f;
    const float viewSpanY = (m_viewSpanY > 0.0f) ? m_viewSpanY : 1.0f;

    float xScale = viewSpanX / xPlotSpan;
    float yScale = viewSpanY / yPlotSpan;
    return {xScale, yScale};
}
inline std::pair<float, float> Plot2D::GetWindowToViewportScaleFactors(const int windowWidth,
                                                                       const int windowHeight)
{
    const float xPlotSpan = windowWidth - 2 * m_plotBorderOffsets.first;
    const float yPlotSpan = windowHeight - 2 * m_plotBorderOffsets.second;

    const float viewSpanX = (m_viewSpanX > 0.0f) ? m_viewSpanX : 1.0f;
    const float viewSpanY = (m_viewSpanY > 0.0f) ? m_viewSpanY : 1.0f;

    float xScale = xPlotSpan / viewSpanX;
    float yScale = yPlotSpan / viewSpanY;
    return {xScale, yScale};
}

Plot2D::GuiPolyline Plot2D::GetDataPolyline(const LineSeries& series, IWindow& window)
{
    WindowRect rect = window.GetRect();
    std::pair<float, float> scales = GetWindowToViewportScaleFactors(rect.right, rect.top);

    std::vector<uint8_t> pixel_mask(rect.top * rect.right, 0);
    std::vector<Point> to_draw;
    to_draw.reserve(series.x.size());

    size_t idx;
    Point transformedPoint;
    std::pair<float, float> point;
    for (int i = 0; i < series.x.size(); i++)
    {
        point = {series.x[i], series.y[i]};
        transformedPoint = {static_cast<int>(((point.first - m_viewZero.first) * scales.first) +
                                             m_plotBorderOffsets.first),
                            static_cast<int>(((point.second - m_viewZero.second) * scales.second) +
                                             m_plotBorderOffsets.second)};

        // Bounds-check index before writing mask
        if (transformedPoint.first < m_viewportRect.left || transformedPoint.first >= m_viewportRect.right ||
            transformedPoint.second < m_viewportRect.bottom || transformedPoint.second >= m_viewportRect.top)
            continue;

        // Check if pixel already set in mask. If not, add to draw list and set mask
        idx = transformedPoint.second * rect.right + transformedPoint.first;
        if (!pixel_mask[idx])
        {
            pixel_mask[idx] = 1;
            to_draw.emplace_back(transformedPoint);
        }
    }

    return GuiPolyline(to_draw, series.color);
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
void Plot2D::SetViewportRect(const WindowRect& rect)
{
    m_viewportRect = rect;
}

void Plot2D::OnWindowResizeCallback(IWindow& window)
{
    std::pair<int, int> size = window.GetRect().Size();
    float verticalOffset = size.second * m_plotBorderOffsetFactor;
    float horizontalOffset = size.first * m_plotBorderOffsetFactor;
    SetPlotBorderOffsets(std::pair<float, float>{size.first * m_plotBorderOffsetFactor,
                                                 size.second * m_plotBorderOffsetFactor});
    SetViewportRect(WindowRect(size.second - verticalOffset, horizontalOffset,
                               size.first - horizontalOffset, verticalOffset));
    // TODO: Use binning to reduce number of points drawn for large datasets
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    window.Invalidate(m_plotWindowState);
    
}
void Plot2D::OnWindowResizeEndCallback(IWindow& window)
{
    std::pair<int, int> size = window.GetRect().Size();

    float verticalOffset = size.second * m_plotBorderOffsetFactor;
    float horizontalOffset = size.first * m_plotBorderOffsetFactor;
    SetPlotBorderOffsets(std::pair<float, float>{size.first * m_plotBorderOffsetFactor,
                                                 size.second * m_plotBorderOffsetFactor});
    SetViewportRect(WindowRect(size.second - verticalOffset, horizontalOffset,
                               size.first - horizontalOffset, verticalOffset));
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    window.Invalidate(m_plotWindowState);
}
void Plot2D::OnScrollWheelMoveCallback(ScrollEvent wheelEvent, IWindow& window)
{
    // Temporarily disable mouse hover to avoid blanking during next update
    m_window->OnMouseHoverCallback = nullptr;

    // Choose a sensible sensitivity; make it small so wheel gives smooth zoom.
    const double sensitivity = 0.0015;  // tweak as desired
    double factor = std::exp(wheelEvent.deltaY * sensitivity);

    // Clamp factor to avoid extreme jumps
    if (factor < 0.1) factor = 0.1;
    if (factor > 10.0) factor = 10.0;

    // View center in data coords
    float centerX = m_viewZero.first + m_viewSpanX * 0.5f;
    float centerY = m_viewZero.second + m_viewSpanY * 0.5f;

    // New spans
    float newSpanX = static_cast<float>(m_viewSpanX * factor);
    float newSpanY = static_cast<float>(m_viewSpanY * factor);

    // Optionally clamp minimal span to avoid divide-by-zero / overzoom
    const float minSpan = 1e-6f;
    if (newSpanX < minSpan) newSpanX = minSpan;
    if (newSpanY < minSpan) newSpanY = minSpan;

    // Recenter view around same center
    m_viewZero.first = centerX - newSpanX * 0.5f;
    m_viewZero.second = centerY - newSpanY * 0.5f;
    m_viewSpanX = newSpanX;
    m_viewSpanY = newSpanY;

    // Update and redraw
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    window.Invalidate(m_plotWindowState);

    // Re-enable mouse hover
    m_window->OnMouseHoverCallback = [this](Point p) { this->OnMouseHoverCallback(*m_window, p); };
}

std::pair<float, float> Plot2D::GetTransformedCoordinates(const int& x, const int& y,
                                                                         const int windowWidth,
                                                                         const int& windowHeight)
{
    std::pair<float, float> scales = GetViewportToWindowScaleFactors(windowWidth, windowHeight);
    auto coordinates =  std::pair<float, float>(
        (x - m_plotBorderOffsets.first) * scales.first + m_viewZero.first,
        ((y - m_plotBorderOffsets.second) * scales.second + m_viewZero.second));

    return coordinates;
}

void Plot2D::OnMouseHoverCallback(IWindow& w, Point mousePos)
{
    std::pair<float, float> plotBorderOffsets = GetPlotBorderOffsets();
    WindowRect rect = w.GetRect();
    bool inside = (mousePos.first >= plotBorderOffsets.first &&
                   mousePos.first <= (rect.Size().first) - plotBorderOffsets.first) &&
                  (mousePos.second >= plotBorderOffsets.second &&
                   mousePos.second <= (rect.Size().second) - plotBorderOffsets.second);

    // Start mouse coordinate text at top left corner with small offset
    int x = rect.left + 5;
    int y = rect.top - 2;

    // Mouse coordinates should fit inside a rect that starts at the top left corner and extends
    // 250 wide and 30 high
    WindowRect mouseCoordinateRect(rect.top, rect.left, rect.left + 250, rect.top - 30); 

    if (inside)
    {
        auto transformedCoords = GetTransformedCoordinates(mousePos.first, mousePos.second,
                                                           rect.Size().first, rect.Size().second);

        char buf[64];
        int n = _snprintf_s(buf, sizeof(buf), _TRUNCATE, "(X, Y) = (%.3g, %.3g)",
                            transformedCoords.first, transformedCoords.second);
        std::string coordText(buf, n > 0 ? n : 0);
        
        WindowState mouseCoordinates;
        mouseCoordinates.text = {
            GuiText(coordText, Point(x, y), 1, Color::White(), Orientation::HORIZONTAL)};
        mouseCoordinates.background = Color::Black();
        w.InvalidateRegion(mouseCoordinateRect, mouseCoordinates);
        m_invalidateMouseCoordRegion = true;
    }
    else if (m_invalidateMouseCoordRegion)
    {
        // Clear previous mouse coordinate region only once when mouse leaves plot area
        m_invalidateMouseCoordRegion = false;
        WindowState empty;
        empty.background = Color::Black();
        w.InvalidateRegion(mouseCoordinateRect, empty);
    }
}

void Plot2D::OnSaveButtonClicked(IWindow& w)
{
    w.SaveScreenshotAsPNG(title + "_" + w.GetTimestamp());
}
void Plot2D::OnResetZoomButtonClicked(IWindow& w)
{
    // Temporarily disable mouse hover to avoid blanking during next update
    m_window->OnMouseHoverCallback = nullptr;

    // Restore saved defaults
    m_viewZero = m_defaultViewZero;
    m_viewSpanX = m_defaultViewSpanX;
    m_viewSpanY = m_defaultViewSpanY;

    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    w.Invalidate(m_plotWindowState);

    // Re-enable mouse hover
    m_window->OnMouseHoverCallback = [this](Point p) { this->OnMouseHoverCallback(*m_window, p); };
}

}  // namespace cpplot2d