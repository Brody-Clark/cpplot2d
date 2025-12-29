/**********************************************************************************************
 *
 *                                    ░██               ░██     ░██████         ░██
 *                                    ░██               ░██    ░██   ░██        ░██
 *   ░███████  ░████████  ░████████   ░██  ░███████  ░████████       ░██  ░████████
 *   ░██    ░██ ░██    ░██ ░██    ░██ ░██ ░██    ░██    ░██      ░█████  ░██    ░██
 *   ░██        ░██    ░██ ░██    ░██ ░██ ░██    ░██    ░██     ░██      ░██    ░██
 *   ░██    ░██ ░███   ░██ ░███   ░██ ░██ ░██    ░██    ░██    ░██       ░██   ░███
 *   ░███████  ░██░█████  ░██░█████   ░██  ░███████      ░████ ░████████  ░█████░██
 *             ░██        ░██
 *             ░██        ░██
 *
 *
 *  cpplot2d v1.0.0
 *
 *  A cross-platform, single-header, C++ plotting library with zero third-party dependencies.
 *
 *  Copyright (c) 2025 BRODY CLARK
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 **********************************************************************************************/
#pragma once

#define CPPLOT2D_VERSION_MAJOR 1
#define CPPLOT2D_VERSION_MINOR 0
#define CPPLOT2D_VERSION_PATCH 0
#define CPPLOT2D_VERSION  "1.0.0"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <optional>
#include <cstring>
#include <cstdint>
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
#endif

#ifdef CPPLOT2D_ENABLE_DEBUG
// Print to stderr and, on Windows, send to the debugger output.
#define CPPLOT2D_DEBUG(fmt, ...)                                                           \
    do                                                                                     \
    {                                                                                      \
        std::fprintf(stderr, "[cpplot2d DEBUG] %s:%d:%s(): " fmt "\n", __FILE__, __LINE__, \
                     __func__, ##__VA_ARGS__);                                             \
        std::fflush(stderr);                                                               \
        /* also send to Visual Studio Output window */                                     \
        /* small, non-portable helper */                                                   \
        /* Note: OutputDebugString expects a wide string on Unicode builds; */             \
        /* using ANSI here for simplicity (safe in most dev scenarios). */                 \
        /* If you need wide strings, convert fmt+args into a std::string. */               \
        /* Keep this call optional to avoid depending on windows.h globally. */            \
        do                                                                                 \
        {                                                                                  \
        } while (0);                                                                       \
    } while (0)
#else
// No-op in release builds
#define CPPLOT2D_DEBUG(fmt, ...) ((void)0)
#endif

#ifdef _WIN32
#include <commdlg.h>
#include <gdiplus.h>  // This must be inlcuded after windows.h
#include <shlobj.h>
#define NOMINMAX
#undef max
#undef min
#pragma comment(lib, "gdiplus.lib")
#elif defined(__linux__)
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#elif defined(__APPLE__)

#ifdef __OBJC__
// NOLINTBEGIN(readability-*, modernize-*, bugprone-*)
#import <Cocoa/Cocoa.h>
#import <Foundation/Foundation.h>

@class WindowView;
@class MenuCallbackBridge;
@class WindowDelegate;
// NOLINTEND(readability-*, modernize-*, bugprone-*)
#endif  // __OBJC__
#endif  // OS check

namespace cpplot2d
{
// Color struct representing RGBA color values.
struct Color
{
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t a = 255;  // alpha, default opaque

    constexpr Color() = default;
    constexpr Color(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha = 255)
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
    static constexpr Color Yellow()
    {
        return {255, 255, 0};
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

namespace detail
{
// 2D integer points (x, y)
using Point = std::pair<int, int>;

// 2D float points (x, y)
using Pointf = std::pair<float, float>;

// 2D dimension (x, y)
using Dimension2d = std::pair<int, int>;

// 2D floating point dimension (x, y)
using Dimension2df = std::pair<float, float>;

// Text orientation
enum class Orientation : uint8_t
{
    HORIZONTAL = 0,
    VERTICAL = 1
};

// Text alignment
enum class Alignment : uint8_t
{
    LEFT = 0,
    RIGHT = 1
};

// Represents a polyline to be drawn in the window. More efficient than multiple lines.
struct GuiPolyline
{
   public:
    GuiPolyline(std::vector<Point> points = {}, Color c = Color::Green(), int thickness = 1)
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
    GuiLine() = default;
    Point p1 = {-1, -1};
    Point p2 = {-1, -1};
    Color color = Color::White();
};

// Represents a text entry to be drawn in the window
struct GuiText
{
   public:
    GuiText(const std::string& text, const Point pos, const Color& color, Orientation orientation,
            int size = 10, std::string font = "", Alignment alignment = Alignment::LEFT)
        : text(text),
          pos(pos),
          color(color),
          orientation(orientation),
          size(size),
          font(font),
          alignment(alignment)
    {
    }
    GuiText() = default;
    std::string text = "";
    Point pos = {-1, -1};
    int size = 10;
    std::string font = "";
    Color color = Color::White();
    Orientation orientation = Orientation::HORIZONTAL;
    Alignment alignment = Alignment::LEFT;
};

// Represents a rectangle to be drawn in the window
struct GuiRect
{
   public:
    GuiRect(Point topLeft, Point bottomRight, Color borderColor, int borderWidth = 1)
        : topLeft(topLeft),
          bottomRight(bottomRight),
          borderColor(borderColor),
          borderWidth(borderWidth)
    {
    }
    GuiRect() = default;
    Point topLeft = {-1, -1};
    Point bottomRight = {-1, -1};
    Color borderColor = Color::White();
    int borderWidth = 1;
};

struct GuiFrame
{
   public:
    GuiFrame(Point topLeft, Point bottomRight, Color borderColor, int borderWidth = 1)
        : topLeft(topLeft),
          bottomRight(bottomRight),
          borderColor(borderColor),
          borderWidth(borderWidth)
    {
    }
    GuiFrame() = default;
    Point topLeft = {-1, -1};
    Point bottomRight = {-1, -1};
    Color borderColor = Color::White();
    int borderWidth = 1;
};

// Represents a circle to be drawn in the window
struct GuiCircle
{
   public:
    GuiCircle(Point center, int radius, Color fillColor)
        : center(center), radius(radius), fillColor(fillColor)
    {
    }
    GuiCircle() = default;

    Point center = {-1, -1};
    int radius = 0;
    Color fillColor = Color::Black();
};

// Represents the entire state of the window to be drawn
struct WindowState
{
    std::vector<GuiText> text = {};
    std::vector<GuiLine> lines = {};
    std::vector<GuiPolyline> polylines = {};
    std::vector<GuiRect> rects = {};
    std::vector<GuiCircle> circles = {};
    Color background = Color::Black();
    Point minSize = {200, 100};
};
}  // namespace detail
}  // namespace cpplot2d

#ifdef __APPLE__
#ifdef __OBJC__
// NOLINTBEGIN(readability-*, modernize-*, bugprone-*)
// Typedefs in the global namespace
typedef cpplot2d::detail::GuiLine GuiLine;
typedef cpplot2d::detail::GuiPolyline GuiPolyline;
typedef cpplot2d::detail::GuiCircle GuiCircle;
typedef cpplot2d::detail::GuiText GuiText;
typedef cpplot2d::detail::GuiRect GuiRect;
typedef cpplot2d::detail::WindowState WindowState;

@interface WindowView : NSView
@property(nonatomic, assign) WindowState* windowState;
@property(nonatomic, strong) NSTrackingArea* trackingArea;
@end
extern NSString* const MouseMovedNotification = @"MouseMovedNotification";

@interface MenuCallbackBridge : NSObject
{
    std::function<void()> _callback;
}
- (instancetype)initWithCallback:(std::function<void()>)callback;
- (void)invokeCallback:(id)sender;
@end

@implementation MenuCallbackBridge
- (instancetype)initWithCallback:(std::function<void()>)callback
{
    self = [super init];
    if (self)
    {
        _callback = callback;
    }
    return self;
}

- (void)invokeCallback:(id)sender
{
    if (_callback)
    {
        _callback();
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

@implementation WindowView
- (instancetype)initWithFrame:(NSRect)frame
{
    self = [super initWithFrame:frame];
    if (self)
    {
        [self.window setAcceptsMouseMovedEvents:YES];
    }
    return self;
}
- (void)viewDidMoveToWindow
{
    [super viewDidMoveToWindow];

    if (self.window)
    {
        [self.window setAcceptsMouseMovedEvents:YES];
    }

    // Call updateTrackingAreas to ensure tracking is set up once the window is present
    [self updateTrackingAreas];
}
NSColor* NSColorFromCppColor(const cpplot2d::Color& cppColor)
{
    // Perform the scaling conversion for each component
    CGFloat red = (CGFloat)cppColor.r / 255.0f;
    CGFloat green = (CGFloat)cppColor.g / 255.0f;
    CGFloat blue = (CGFloat)cppColor.b / 255.0f;
    CGFloat alpha = (CGFloat)cppColor.a / 255.0f;

    // Use the AppKit NSColor initializer
    return [NSColor colorWithRed:red green:green blue:blue alpha:alpha];
}
- (void)drawCircleAt:(NSPoint)center radius:(CGFloat)radius color:(NSColor*)color
{
    NSRect circleRect = NSMakeRect(center.x - radius, center.y - radius, radius * 2, radius * 2);
    NSBezierPath* circlePath = [NSBezierPath bezierPathWithOvalInRect:circleRect];

    [color setFill];
    [circlePath fill];
}
- (void)drawRectWithTopLeft:(NSPoint)topLeft bottomRight:(NSPoint)bottomRight color:(NSColor*)color
{
    NSRect rect =
        NSMakeRect(topLeft.x, bottomRight.y, bottomRight.x - topLeft.x, topLeft.y - bottomRight.y);

    NSBezierPath* path = [NSBezierPath bezierPathWithRect:rect];

    [color setStroke];
    [path stroke];
}
- (NSFont*)getFontWithName:(NSString*)fontName size:(int)size
{
    NSFont* font = nil;

    // Try requested font if provided
    if (fontName.length > 0)
    {
        font = [NSFont fontWithName:fontName size:size];
    }

    // Fallback to system font
    if (!font)
    {
        font = [NSFont systemFontOfSize:size];
    }

    return font;
}
- (void)drawVerticalText:(NSString*)text
                 atPoint:(NSPoint)point
                   color:(NSColor*)color
                fontName:(NSString*)fontName
                    size:(int)size
{
    NSFont* customFont = [self getFontWithName:fontName size:size];
    NSDictionary* attributes =
        @{NSFontAttributeName : customFont, NSForegroundColorAttributeName : color};

    // Save the current graphics context
    NSGraphicsContext* context = [NSGraphicsContext currentContext];
    [context saveGraphicsState];

    // Apply a rotation transform
    NSAffineTransform* transform = [NSAffineTransform transform];
    [transform translateXBy:point.x yBy:point.y];
    [transform rotateByDegrees:90];  // Rotate 90 degrees for vertical text
    [transform concat];

    // Draw the text at the transformed position
    [text drawAtPoint:NSMakePoint(0, 0) withAttributes:attributes];

    // Restore the graphics context
    [context restoreGraphicsState];
}
- (void)drawRightAlignedText:(NSString*)text
                     atPoint:(NSPoint)point
                       color:(NSColor*)color
                    fontName:(NSString*)fontName
                        size:(int)size
{
    NSFont* customFont = [self getFontWithName:fontName size:size];
    NSMutableParagraphStyle* style = [[NSMutableParagraphStyle alloc] init];
    [style setAlignment:NSTextAlignmentRight];

    NSDictionary* attributes = @{
        NSFontAttributeName : customFont,
        NSForegroundColorAttributeName : color,
        NSParagraphStyleAttributeName : style
    };

    // Adjust the X coordinate
    // subtract the width from the anchor point so the 'end' of the text is at anchorPoint.x
    NSSize textSize = [text sizeWithAttributes:attributes];
    NSPoint adjustedPoint = NSMakePoint(point.x - textSize.width, point.y);

    [text drawAtPoint:adjustedPoint withAttributes:attributes];
}

- (void)drawText:(NSString*)text
         atPoint:(NSPoint)point
           color:(NSColor*)color
        fontName:(NSString*)fontName
            size:(int)size
{
    if (!text) return;
    NSFont* font  = [self getFontWithName:fontName size:size];

    NSDictionary* attributes =
        @{NSFontAttributeName : font, NSForegroundColorAttributeName : color};

    [text drawAtPoint:point withAttributes:attributes];
}
- (void)drawPolyline:(NSArray<NSValue*>*)points color:(NSColor*)color
{
    if (points.count < 2) return;  // Need at least two points to draw a line

    NSBezierPath* path = [NSBezierPath bezierPath];
    [path moveToPoint:points[0].pointValue];  // Start at the first point

    for (int i = 1; i < points.count; i++)
    {
        [path lineToPoint:points[i].pointValue];  // Connect to the next point
    }

    [color setStroke];
    [path setLineWidth:2.0];
    [path stroke];
}
- (void)drawLineFrom:(NSPoint)start to:(NSPoint)end color:(NSColor*)color
{
    // Set the stroke color
    [[NSColor whiteColor] setStroke];

    NSValue* line = [NSValue valueWithBytes:&start objCType:@encode(NSPoint)];
    NSValue* endLine = [NSValue valueWithBytes:&end objCType:@encode(NSPoint)];

    // Create a path for the line
    NSBezierPath* path = [NSBezierPath bezierPath];
    [path moveToPoint:start];  // Start point
    [path lineToPoint:end];    // End point

    // Stroke the path (draw the line)
    [path stroke];
}

- (void)drawRect:(NSRect)dirtyRect
{
    [super drawRect:dirtyRect];

    // Use the windowState to draw the content
    if (self.windowState)
    {
        // Clear the background
        [NSColorFromCppColor(self.windowState->background) setFill];
        NSRectFill(dirtyRect);

        // Draw lines
        for (const GuiLine& line : self.windowState->lines)
        {
            [self drawLineFrom:NSMakePoint(line.p1.first, line.p1.second)
                            to:NSMakePoint(line.p2.first, line.p2.second)
                         color:NSColorFromCppColor(line.color)];
        }

        // Draw polylines
        for (const GuiPolyline& polyline : self.windowState->polylines)
        {
            NSMutableArray<NSValue*>* points = [NSMutableArray array];
            for (const cpplot2d::detail::Point& p : polyline.points)
            {
                [points addObject:[NSValue valueWithPoint:NSMakePoint(p.first, p.second)]];
            }
            [self drawPolyline:points color:NSColorFromCppColor(polyline.color)];
        }

        // Draw circles
        for (const GuiCircle& circle : self.windowState->circles)
        {
            [self drawCircleAt:NSMakePoint(circle.center.first, circle.center.second)
                        radius:circle.radius
                         color:NSColorFromCppColor(circle.fillColor)];
        }

        // Draw rects
        for (const GuiRect& rect : self.windowState->rects)
        {
            [self drawRectWithTopLeft:NSMakePoint(rect.topLeft.first, rect.topLeft.second)
                          bottomRight:NSMakePoint(rect.bottomRight.first, rect.bottomRight.second)
                                color:NSColorFromCppColor(rect.borderColor)];
        }

        // Draw text
        for (const GuiText& text : self.windowState->text)
        {
            if (text.orientation == cpplot2d::detail::Orientation::HORIZONTAL)
            {
                if (text.alignment == cpplot2d::detail::Alignment::LEFT)
                {
                    [self drawText:[NSString stringWithUTF8String:text.text.c_str()]
                           atPoint:NSMakePoint(text.pos.first, text.pos.second)
                             color:NSColorFromCppColor(text.color)
                          fontName:[NSString stringWithUTF8String:text.font.c_str()]
                              size:text.size];
                }
                else
                {
                    [self drawRightAlignedText:[NSString stringWithUTF8String:text.text.c_str()]
                                       atPoint:NSMakePoint(text.pos.first, text.pos.second)
                                         color:NSColorFromCppColor(text.color)
                                      fontName:[NSString stringWithUTF8String:text.font.c_str()]
                                          size:text.size];
                }
            }
            else if (text.orientation == cpplot2d::detail::Orientation::VERTICAL)
            {
                [self drawVerticalText:[NSString stringWithUTF8String:text.text.c_str()]
                               atPoint:NSMakePoint(text.pos.first, text.pos.second)
                                 color:NSColorFromCppColor(text.color)
                              fontName:[NSString stringWithUTF8String:text.font.c_str()]
                                  size:text.size];
            }
        }
    }
}
- (void)updateTrackingAreas
{
    [super updateTrackingAreas];

    // Remove any existing tracking areas
    for (NSTrackingArea* area in [self trackingAreas])
    {
        [self removeTrackingArea:area];
    }
    self.trackingArea = nil;

    // Create a new tracking area
    NSTrackingArea* trackingArea =
        [[NSTrackingArea alloc] initWithRect:self.bounds
                                     options:(NSTrackingMouseEnteredAndExited |
                                              NSTrackingMouseMoved | NSTrackingActiveAlways |
                                              NSTrackingInVisibleRect | NSTrackingActiveInKeyWindow)
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
}
- (void)resetCursorRects
{
    [self addCursorRect:[self bounds] cursor:[NSCursor resizeLeftRightCursor]];
}

- (void)viewDidChangeBackingProperties
{
    [super viewDidChangeBackingProperties];
    [self setNeedsDisplay:YES];
}
- (void)setFrameSize:(NSSize)newSize
{
    [super setFrameSize:newSize];
    [self setNeedsDisplay:YES];
}
- (void)SaveScreenshot:(const std::string&)fileName
{
    // Show a save dialog
    NSSavePanel* savePanel = [NSSavePanel savePanel];
    NSArray* fileTypes = [NSArray arrayWithObjects:@"png", nil];
    savePanel.allowedContentTypes = fileTypes;

    NSString* defaultName = [NSString stringWithUTF8String:fileName.c_str()];
    [savePanel setNameFieldStringValue:defaultName];

    if ([savePanel runModal] == NSModalResponseOK)
    {
        NSURL* saveURL = [savePanel URL];

        // Render view into an image
        NSBitmapImageRep* imageRep = [self bitmapImageRepForCachingDisplayInRect:self.bounds];
        [self cacheDisplayInRect:self.bounds toBitmapImageRep:imageRep];

        // Convert to PNG data
        NSData* pngData =
            [imageRep representationUsingType:NSBitmapImageFileTypePNG properties:@{}];

        // Write to file
        [pngData writeToURL:saveURL atomically:YES];
    }
}
@end
// NOLINTEND(readability-*, modernize-*, bugprone-*)
#endif  // __OBJC__
#endif  // __APPLE__

namespace cpplot2d
{

// Plot appearance properties
struct PlotProperties
{
   public:
    Color backgroundColor = Color::Black();
    Color borderColor = Color::White();
    int tickLineCount = 4;
    const std::pair<int, int> minWindowSize = {200, 100};
    const std::pair<int, int> defaultWindowSize = {800, 600};
    std::string font = "Tahoma";
};

class Plot2D
{
   public:
    Plot2D(std::string title = "", std::string xLabel = "", std::string yLabel = "",
           PlotProperties props = {});

    /**
     Adds a line series to the plot.
     @tparam T Numeric type of the input data (e.g., float, double, int)
     @param x Vector of x-coordinates
     @param y Vector of y-coordinates
     @param color Color of the line (default: green)
     @param size Thickness of the line (default: 1)
    */
    template <typename T>
    Plot2D& AddLine(const std::vector<T>& x, const std::vector<T>& y, Color color = Color::Green(),
                    int size = 1);

    /**
     Show the plot with the pre-determined plot points and parameters.
     Set block to false to prevent blocking the main thread (e.g. when created from within another
     gui app).
    */
    void Show(bool block = true);

    /**
     Sets whether the legend should be shown on the plot or not.

     @param show whether or not to show the legend
    */
    void DisplayLegend(bool show);

   protected:
    using Point = detail::Point;
    using Pointf = detail::Pointf;
    using Dimension2d = detail::Dimension2d;
    using Dimension2df = detail::Dimension2df;
    using GuiLine = detail::GuiLine;
    using GuiPolyline = detail::GuiPolyline;
    using GuiText = detail::GuiText;
    using GuiCircle = detail::GuiCircle;
    using GuiRect = detail::GuiRect;
    using WindowState = detail::WindowState;
    using Orientation = detail::Orientation;
    using Alignment = detail::Alignment;
    using MenuButtons = std::map<std::string, std::function<void()>>;

    // Plot interaction modes
    enum class InteractionMode : uint8_t
    {
        NONE = 0,
        PAN_DEFAULT = 1,
        ZOOM_DEFAULT = 2,
        PAN_ACTIVE = 3,
        ZOOM_ACTIVE = 4
    };

    // Data series
    class Series
    {
       public:
        Series(const std::vector<float>& xs, const std::vector<float>& ys)
        {
            data.reserve(xs.size());
            for (int i = 0; i < xs.size(); i++)
            {
                data.emplace_back(Pointf{xs[i], ys[i]});
            }
        }
        std::vector<Pointf> data;
    };

    // Series for line plots
    class LineSeries : public Series
    {
       public:
        LineSeries(const std::vector<float>& xs, const std::vector<float>& ys, Color color,
                   int size = 1)
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
        ScatterSeries(const std::vector<float>& xs, const std::vector<float>& ys, Color color,
                      int pointSize = 1)
            : Series(xs, ys), pointSize(pointSize), color(color)
        {
        }
        int pointSize;
        Color color;
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
     *   - This library uses a universal coordinate system where origin is at the
     *     bottom-left for drawing logic
     */
    struct WindowRect
    {
       public:
        WindowRect(int top, int left, int right, int bottom)
            : top(top), bottom(bottom), right(right), left(left)
        {
        }
        WindowRect() = default;

        const Dimension2d Size() const
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

    // Interface for platform-specific window implementations
    class IWindow
    {
       public:
        virtual ~IWindow() = default;

        // Invalidates entire window, forcing a redraw
        virtual void Invalidate(WindowState* windowState) = 0;

        // Returns average character width in pixels for the current font
        virtual Dimension2d GetAverageCharSize() = 0;

        // Invalidates only the space  encompassed in the given rect
        virtual void InvalidateRegion(const WindowRect& rect, WindowState* windowState) = 0;

        // Adds a button to the menu bar
        virtual void AddMenuButtons(const std::string menu, MenuButtons menuButtons) = 0;

        // Saves a screenshot of the window as an image file. fileName is the name of the file only.
        virtual bool SaveScreenshot(const std::string& fileName) = 0;

        // Sets whether the window is visible or hidden
        virtual void SetIsVisible(bool isVisible) = 0;

        // Returns a timestamp string for use in file names in the format YYYYMMDD_HHMMSS
        virtual std::string GetTimestamp() = 0;

        // Returns the current window rectangle
        virtual WindowRect GetRect() = 0;

        // Runs the window event loop (blocking)
        virtual void RunEventLoop() = 0;

        // Callbacks
        std::function<void(Point)> OnMouseMoveCallback;
        std::function<void(Point)> OnMouseLButtonDownCallback;
        std::function<void(Point)> OnMouseLButtonUpCallback;
        std::function<void()> OnResizeStartCallback;
        std::function<void()> OnResizeEndCallback;
        std::function<void()> OnResizeCallback;
    };

    // Interface for platform-specific graphics context implementations
    class IGraphicsContext
    {
       public:
        virtual ~IGraphicsContext() = default;
        virtual void Init() = 0;
        virtual void Shutdown() = 0;
        virtual Plot2D::IWindow* MakeWindow(WindowState* initialState, Dimension2d defaultSize,
                                            Dimension2d minSize, bool isVisible,
                                            const std::string& title) = 0;
    };

    InteractionMode m_InteractionMode = InteractionMode::NONE;
    static std::unique_ptr<IGraphicsContext> m_graphicsContext;
    std::unique_ptr<IWindow> m_window;

    // State representing what to draw in the plot window
    std::unique_ptr<WindowState> m_plotWindowState = std::make_unique<WindowState>();
    // Separate state for mouse coordinates to avoid redrawing everything
    std::unique_ptr<WindowState> m_coordinateViewState = std::make_unique<WindowState>();

    void Initialize();
    void UpdateOffsets(const std::vector<float>& x, const std::vector<float>& y);
    void OnMouseMoveCallback(IWindow& window, Point mousePos);
    void OnMouseLButtonDownCallback(IWindow& window, Point mousePos);
    void OnMouseLButtonUpCallback(IWindow& window, Point mousePos);
    void OnWindowResizeCallback(IWindow& window);
    void OnSaveClicked(IWindow& window);
    void OnToggleZoomClicked(IWindow& window);
    void OnToggleGrabClicked(IWindow& window);
    void OnResetViewClicked(IWindow& window);
    void HandleMouseHover(IWindow& w, Point mousePos);
    void HandleZoomDrag(IWindow& w, Point mousePos);
    GuiText GetInteractionText(const std::string& label, WindowRect rect);
    void HandlePanDrag(IWindow& w, Point mousePos);
    void Zoom(WindowRect rect, IWindow& w);
    const bool IsPointInsideRect(const Point& p, const WindowRect& rect);
    Pointf GetPlotBorderOffsets();
    void SetViewportRect(const WindowRect& rect);
    void SetPlotBorderOffsets(Pointf offsets);
    void UpdateViewportWindowScaleFactors(const Dimension2d& windowSize);
    Pointf GetViewportToWindowScaleFactor(const Dimension2d& windowSize);
    Pointf GetTransformedCoordinates(Point coord);
    void UpdatePlotWindowState(WindowState* windowState, IWindow& window);
    void InitializePlotState(WindowState* windowState);
    void GetDataPolyline(const LineSeries& series, const WindowRect&, GuiPolyline& output);
    Point GetIntersectionPointOnRect(const Point& p1, const Point& p2, const WindowRect& rect);
    int GetIntersectionPointsOnRect(const Point& p1, const Point& p2, const WindowRect& rect,
                                    Point& out1, Point& out2);
    int DistanceSquared(const Point& p1, const Point& p2);
    int64_t Cross(const Point& a, const Point& b, const Point& c);
    bool onSegment(const Point& a, const Point& b, const Point& c);
    bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4);
    bool IntersectionPoint(const Point& a, const Point& b, const Point& c, const Point& d,
                           Point& result);
    GuiRect GetPlotBorderRect(const WindowRect&, Color color);
    std::pair<std::vector<GuiLine>, std::vector<GuiText>> GetPlotBorderTickLines(const WindowRect&,
                                                                                 Color color);
    std::vector<GuiText> GetPlotLabels(const WindowRect&, Color color);

   private:
    struct m_dataSeries
    {
        std::vector<LineSeries> lines = {};
        std::vector<ScatterSeries> points = {};
    } m_dataSeries;
    std::vector<Point> m_polylineBuffer = {};
    Dimension2df m_plotZeroOffsets = {(std::numeric_limits<float>::max)(),
                                      (std::numeric_limits<float>::max)()};
    Dimension2df m_plotBorderOffsets = {0.f, 0.f};  // offsets from window edge to plot area

    WindowRect m_viewportRect = {0, 0, 0, 0};  // current viewport in window space
    const std::string m_font = "Tahoma";
    // scale factors to convert viewport coords to window coords
    std::pair<float, float> m_viewportToWindowScaleFactors = {1.0f, 1.0f};
    // scale factors to convert window coords to viewport coords
    std::pair<float, float> m_windowToViewportScaleFactors = {1.0f, 1.0f};

    // Runtime view (what is currently shown)
    Pointf m_viewZero = {0.0f, 0.0f};  // data-space lower-left of current view
    float m_viewSpanX = 0.0f;          // data-space width of current view
    float m_viewSpanY = 0.0f;          // data-space height of current view

    // Whether to invalidate small region for mouse coords.
    // Useful to prevent invalidating more than once.
    bool m_invalidateMouseCoordRegion = false;
    Point m_lastMousePos = {0, 0};
    std::pair<float, float> m_defaultViewZero = {0.0f, 0.0f};
    float m_defaultViewSpanX = 0.0f;
    float m_defaultViewSpanY = 0.0f;
    Dimension2d m_charSize = {5, 5};
    static constexpr float m_plotBorderOffsetFactor = 0.125f;
    std::string xLabel = "";
    std::string yLabel = "";
    std::string title = "";
    Dimension2d m_mouseCoordinateRectOffset = {200, 15};
    std::pair<float, float> m_dataSpans = {0.0f, 0.0f};
    std::pair<float, float> m_largestDataPoints = {-(std::numeric_limits<float>::max)(),
                                                   -(std::numeric_limits<float>::max)()};
    std::pair<float, float> m_smallestDataPoints = {(std::numeric_limits<float>::max)(),
                                                    (std::numeric_limits<float>::max)()};
    static constexpr int m_tickLength = 4;
    static constexpr std::pair<int, int> minWindowSize = {200, 100};
    Dimension2d defaultWindowSize = {800, 600};
    PlotProperties m_plotProperties;

#ifdef CPPLOT2D_HEADLESS  // Null/Headless implementation
    class NullWindow : public cpplot2d::Plot2D::IWindow
    {
       public:
        NullWindow() = default;
        ~NullWindow() override = default;
        void Invalidate(WindowState* windowState) override;
        Dimension2d GetAverageCharSize() override;
        void InvalidateRegion(const WindowRect& rect, WindowState* windowState) override;
        void AddMenuButtons(const std::string menu, MenuButtons menuButtons) override;
        bool SaveScreenshot(const std::string& fileName) override;
        void SetIsVisible(bool isVisible) override;
        std::string GetTimestamp() override;
        WindowRect GetRect() override;
        void RunEventLoop() override;
    };

    class NullGraphicsContext : public cpplot2d::Plot2D::IGraphicsContext
    {
       public:
        void Init() override;
        void Shutdown() override;
        IWindow* MakeWindow(WindowState* initialState, Dimension2d defaultSize, Dimension2d minSize,
                            bool isVisible, const std::string& title) override;
    };
#endif
#ifdef _WIN32  // Windows-specific implementation
    class Win32GraphicsContext : public cpplot2d::Plot2D::IGraphicsContext
    {
       public:
        void Init() override;
        void Shutdown() override;
        IWindow* MakeWindow(WindowState* initialState, Dimension2d defaultSize, Dimension2d minSize,
                            bool isVisible, const std::string& title) override;

       protected:
        ULONG_PTR m_gdiplusToken = 0;
    };

    class Win32Window : public cpplot2d::Plot2D::IWindow
    {
       public:
        Win32Window(Dimension2d defaultWindowSize, Dimension2d pos, std::string title,
                    WindowState* initialState);
        ~Win32Window() override = default;

        Dimension2d GetAverageCharSize() override;
        void AddMenuButtons(const std::string menu, MenuButtons menuButtons) override;
        void SetIsVisible(bool isVisible) override;
        void Invalidate(WindowState* windowState) override;
        std::string GetTimestamp() override;
        void RunEventLoop() override;
        void InvalidateRegion(const WindowRect& rect, WindowState* windowState) override;
        bool BrowseForFolder(std::string& outFolder);
        WindowRect GetRect() override;
        bool SaveScreenshot(const std::string& fileName) override;
        static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

       private:
        WindowState* m_windowState;
        HWND m_hwnd = nullptr;
        HMENU m_hMenu = nullptr;
        RECT m_invalidatedRegion;
        std::map<int, std::function<void()>> m_menuCommands;
        bool m_drawnOnce = false;

        void DrawWindowState(const RECT& clientRect, const RECT& invalidatedRect);
        LRESULT HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
        void SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename);
        HBITMAP CaptureWindowContent(HWND hwnd);
        COLORREF ToWin32Color(const Color color);
        void DoDrawText(HDC hdc, GuiText text, RECT clientRect);
        HFONT CreateVerticalFont(int height, const std::string& font);
        int GetTextHeight(HDC hdc, int pointSize);
        HFONT CreateFontOfSize(int height, const std::string& font);
        void ResizeBackBuffer(HWND hwnd);
        void DrawBitmap();
        bool m_useCachedBitmap = false;
        bool m_inMove = false;
        bool m_inSize = false;
        struct doubleBufferedDC
        {
            HBITMAP backBuffer = nullptr;
            HDC backDC = nullptr;
            int bufferW = 0;
            int bufferH = 0;
        } m_backBuffer;
    };

#elif defined(__linux__)
    class X11GraphicsContext : public cpplot2d::Plot2D::IGraphicsContext
    {
       public:
        void Init() override;
        void Shutdown() override;
        IWindow* MakeWindow(WindowState* initialState, Dimension2d defaultSize, Dimension2d minSize,
                            bool isVisible, const std::string& title) override;
    };

    class X11Window : public cpplot2d::Plot2D::IWindow
    {
       public:
        X11Window(Dimension2d defaultWindowSize, Dimension2d pos, std::string title,
                  WindowState* initialState);
        ~X11Window() override;

        Dimension2d GetAverageCharSize() override;
        void AddMenuButtons(const std::string menu, MenuButtons menuButtons) override;
        void SetIsVisible(bool isVisible) override;
        void Invalidate(WindowState* windowState) override;
        std::string GetTimestamp() override;
        void RunEventLoop() override;
        void InvalidateRegion(const WindowRect& rect, WindowState* windowState) override;
        bool BrowseForFolder(std::string& outFolder);
        WindowRect GetRect() override;
        bool SaveScreenshot(const std::string& fileName) override;

       protected:
        struct DropdownMenuItem
        {
            std::string text;
            std::function<void()> callback;
            WindowRect bounds;
        };
        struct DropdownMenu
        {
            Window win = 0;
            int x = 0;
            int y = 0;
            int width = 0;
            int height = 0;
            std::vector<DropdownMenuItem> items = {};
        };
        struct Menu
        {
            std::string title;
            WindowRect bounds;
            DropdownMenu* dropdown = nullptr;
        };
        struct OpenMenuState
        {
            Menu* menu;
            int selectedIndex;
        };
        Menu CreateMenu(const std::string& title, const MenuButtons& menuButtons);
        void DrawWindowState(const WindowRect& rect, WindowState* windowState);
        void DrawMenuBar();
        void DrawTextVertical(XFontStruct* font, unsigned long background, unsigned long color,
                              int x, int y, const char* str);
        void DrawDropdownMenu(DropdownMenu* dropdownMenu);
        void HandleMenuClick(int x, int y);
        unsigned long ToX11Pixel(const Color& c);
        XFontStruct* GetFontOfSize(int size);
        uint8_t ExtractChannel(unsigned long pixel, unsigned long mask);
        bool LoadFont();
        WindowRect GetContentRect();

       private:
        std::vector<Menu> m_menus;
        Color m_menuColor = Color(220, 220, 220);
        Dimension2d m_windowDimensions;
        const int m_menuBarHeight = 24;
        WindowState* m_windowState = nullptr;
        std::vector<void*> m_menuCallbacks;
        DropdownMenu* m_activeDropdown = nullptr;
        XFontStruct* m_menuFont = nullptr;
        Pixmap m_backBuffer = 0;
        Display* m_display = nullptr;
        ::Window m_window;
        int m_screen;
        GC m_gc;
        Atom m_wmDelete;
        bool m_running = false;
        std::unordered_map<uint32_t, unsigned long> m_colorCache;
        std::unordered_map<int, XFontStruct*> m_sizeToFontCache;
        const std::string m_fontFallbacks[5] = {
            "-misc-fixed-medium-r-normal--13-120-75-75-C-70-iso8859-1", "fixed", "6x13", "8x13",
            "9x15"};
    };
#elif defined(__APPLE__)
    class CocoaGraphicsContext : public cpplot2d::Plot2D::IGraphicsContext
    {
       public:
        void Init() override;
        void Shutdown() override;
        cpplot2d::Plot2D::IWindow* MakeWindow(WindowState* initialState, Dimension2d defaultSize,
                                              Dimension2d minSize, bool isVisible,
                                              const std::string& title) override;
    };

    class CocoaWindow : public cpplot2d::Plot2D::IWindow
    {
       public:
        CocoaWindow(Dimension2d defaultWindowSize, Dimension2d pos, std::string title,
                    WindowState* initialState);
        ~CocoaWindow() override;

        Dimension2d GetAverageCharSize() override;
        void AddMenuButtons(const std::string menu, MenuButtons menuButtons) override;
        void SetIsVisible(bool isVisible) override;
        void Invalidate(WindowState* windowState) override;
        std::string GetTimestamp() override;
        void RunEventLoop() override;
        void InvalidateRegion(const WindowRect& rect, WindowState* windowState) override;
        bool BrowseForFolder(std::string& outFolder);
        WindowRect GetRect() override;
        bool SaveScreenshot(const std::string& fileName) override;

       private:
        WindowState* m_windowState;
        void* m_nsWindow;
        void* m_mainMenu;
        void* m_windowView;
        void* m_resizeObserver;
        void* m_mouseMoveObserver;
        void* m_mouseDownObserver;
        void* m_mouseUpObserver;
        std::vector<void*> m_menuCallbacks;

        void OnMouseLButtonDown(int x, int y);
        void OnMouseLButtonUp(int x, int y);
        void OnResize(int newWidth, int newHeight);
        void OnMouseMove(int x, int y);
    };

#endif

};  // Plot2D

#ifdef CPPLOT2D_IMPLEMENTATION
#ifdef CPPLOT2D_IMPLEMENTATION_DEFINED
#error "CPPLOT2D_IMPLEMENTATION defined more than once"
#endif
#define CPPLOT2D_IMPLEMENTATION_DEFINED

std::unique_ptr<cpplot2d::Plot2D::IGraphicsContext> cpplot2d::Plot2D::m_graphicsContext = nullptr;

cpplot2d::Plot2D::Plot2D(std::string title, std::string xLabel, std::string yLabel,
                         PlotProperties props)
    : xLabel(xLabel), yLabel(yLabel), title(title), m_plotProperties(props)
{
    Initialize();
}

void cpplot2d::Plot2D::Initialize()
{
#ifdef CPPLOT2D_HEADLESS
    m_graphicsContext = std::make_unique<NullGraphicsContext>();
#elif defined(_WIN32)
    m_graphicsContext = std::make_unique<Win32GraphicsContext>();
#elif defined(__APPLE__)
#ifdef __OBJC__
    m_graphicsContext = std::make_unique<CocoaGraphicsContext>();
#else
    throw std::runtime_error("ObjC not found.");
#endif
#elif defined(__linux__)
    m_graphicsContext = std::make_unique<X11GraphicsContext>();
#else
    std::throw(std::exception("Unsupported platform."));
#endif
    // Must initialize plot state before creating
    InitializePlotState(m_plotWindowState.get());

    // Initialize graphics context and create window
    m_graphicsContext->Init();
    m_window = std::unique_ptr<IWindow>(m_graphicsContext->MakeWindow(
        m_plotWindowState.get(), defaultWindowSize, minWindowSize, false, title));

    m_charSize = m_window->GetAverageCharSize();
    m_mouseCoordinateRectOffset = {40 * m_charSize.first, 2 * m_charSize.second};

    // Set up callbacks
    m_window->OnMouseMoveCallback = [this](Point p) { this->OnMouseMoveCallback(*m_window, p); };
    m_window->OnMouseLButtonDownCallback = [this](Point p)
    { this->OnMouseLButtonDownCallback(*m_window, p); };
    m_window->OnMouseLButtonUpCallback = [this](Point p)
    { this->OnMouseLButtonUpCallback(*m_window, p); };
    m_window->OnResizeStartCallback = [this]() { this->OnWindowResizeCallback(*m_window); };
    m_window->OnResizeCallback = [this]() { this->OnWindowResizeCallback(*m_window); };
    m_window->OnResizeEndCallback = [this]() { this->OnWindowResizeCallback(*m_window); };

    // Add menu buttons
    MenuButtons fileMenuButtons;
    fileMenuButtons["Save"] = [this]() { this->OnSaveClicked(*m_window); };
    m_window->AddMenuButtons("File", fileMenuButtons);

    MenuButtons viewMenuButtons;
    viewMenuButtons["Toggle Zoom"] = [this]() { this->OnToggleZoomClicked(*m_window); };
    viewMenuButtons["Toggle Grab"] = [this]() { this->OnToggleGrabClicked(*m_window); };
    viewMenuButtons["Reset View"] = [this]() { this->OnResetViewClicked(*m_window); };
    m_window->AddMenuButtons("View", viewMenuButtons);
}

void cpplot2d::Plot2D::UpdateOffsets(const std::vector<float>& x, const std::vector<float>& y)
{
    // Get largest and smallest points in dataset
    std::pair<float, float> p;
    int size = static_cast<int>(x.size());
    for (int i = 0; i < size; i++)
    {
        p = {x[i], y[i]};
        if (p.first > m_largestDataPoints.first) m_largestDataPoints.first = p.first;
        if (p.first < m_smallestDataPoints.first) m_smallestDataPoints.first = p.first;
        if (p.second > m_largestDataPoints.second) m_largestDataPoints.second = p.second;
        if (p.second < m_smallestDataPoints.second) m_smallestDataPoints.second = p.second;
    }

    // The data span is the difference between the largest and smallest points
    m_dataSpans.first =
        std::max((m_largestDataPoints.first - m_smallestDataPoints.first), m_dataSpans.first);
    m_dataSpans.second =
        std::max((m_largestDataPoints.second - m_smallestDataPoints.second), m_dataSpans.second);

    // The conversion from data-space to plot-space requires both coord systems to start from (0,
    // 0). These values will be used to adjust the data points during transformation to achieve
    // this.
    m_plotZeroOffsets.first = std::min(m_plotZeroOffsets.first, m_smallestDataPoints.first);
    m_plotZeroOffsets.second = std::min(m_plotZeroOffsets.second, m_smallestDataPoints.second);

    m_viewZero = m_plotZeroOffsets;
    m_viewSpanX = m_dataSpans.first > 0.0f ? m_dataSpans.first : 1.0f;
    m_viewSpanY = m_dataSpans.second > 0.0f ? m_dataSpans.second : 1.0f;

    m_defaultViewZero = m_viewZero;
    m_defaultViewSpanX = m_viewSpanX;
    m_defaultViewSpanY = m_viewSpanY;

    // Calculate scale factors for coordinate transformations (only needs to be recalculated on
    // resize)
    UpdateViewportWindowScaleFactors({defaultWindowSize.first, defaultWindowSize.second});
}

template <typename T>
cpplot2d::Plot2D& cpplot2d::Plot2D::AddLine(const std::vector<T>& x, const std::vector<T>& y,
                                            Color color, int size)
{
    static_assert(std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
                  "Plot2D requires a numeric type for T (bool is not allowed)");
    assert(x.size() == y.size());

    std::vector<float> xf(x.begin(), x.end());
    std::vector<float> yf(y.begin(), y.end());

    m_dataSeries.lines.emplace_back(xf, yf, color, size);
    if (xf.size() > m_polylineBuffer.size()) m_polylineBuffer.resize(xf.size());

    // Allocate polylines for all data series lines
    m_plotWindowState->polylines.resize(m_dataSeries.lines.size());
    UpdateOffsets(xf, yf);  // Update plot offsets based on new data to make sure plot fits all data
    return *this;
}

void cpplot2d::Plot2D::InitializePlotState(cpplot2d::Plot2D::WindowState* windowState)
{
    windowState->background = m_plotProperties.backgroundColor;

    // Preallocate space for lines
    // 10 for X and Y axis ticks
    windowState->lines.resize(10);

    // Preallocate space for rects
    // 1 for zoom box, 1 for plot border
    windowState->rects.resize(2);

    // Preallocate space for text entries:
    // 5 for X axis ticks, 5 for Y axis ticks, 3 for labels, 1 for interaction mode info
    windowState->text.resize(14);
    windowState->minSize = minWindowSize;
}
void cpplot2d::Plot2D::UpdatePlotWindowState(cpplot2d::Plot2D::WindowState* windowState,
                                             IWindow& window)
{
    WindowRect rect = window.GetRect();

    // Clear zoom box rect and update plot border
    windowState->rects.back() = {};
    windowState->rects[0] = GetPlotBorderRect(rect, Color::White());

    // Update datasets
    for (size_t i = 0; i < m_dataSeries.lines.size(); i++)
    {
        GetDataPolyline(m_dataSeries.lines[i], rect, windowState->polylines[i]);
    }

    std::pair<std::vector<GuiLine>, std::vector<GuiText>> ticks =
        GetPlotBorderTickLines(rect, Color::White());

    for (int i = 0; i < (int)ticks.first.size(); i++)
    {
        windowState->lines[i] = ticks.first[i];
    }
    for (int i = 0; i < (int)ticks.second.size(); i++)
    {
        windowState->text[i] = ticks.second[i];
    }

    std::vector<GuiText> labels = GetPlotLabels(rect, Color::White());
    for (int i = (int)ticks.second.size(); i < (int)ticks.second.size() + (int)labels.size(); i++)
    {
        windowState->text[i] = labels[i - (int)ticks.second.size()];
    }

    // Update position of interaction mode label
    windowState->text.back() = GetInteractionText(windowState->text.back().text, rect);
}

inline cpplot2d::Plot2D::GuiRect cpplot2d::Plot2D::GetPlotBorderRect(const WindowRect& rect,
                                                                     Color color)
{
    const int leftBorderPos = rect.left + (int)m_plotBorderOffsets.first;
    const int rightBorderPos = rect.right - (int)m_plotBorderOffsets.first;
    const int topBorderPos = rect.top - (int)m_plotBorderOffsets.second;
    const int bottomBorderPos = rect.bottom + (int)m_plotBorderOffsets.second;

    return GuiRect({leftBorderPos, topBorderPos}, {rightBorderPos, bottomBorderPos}, color, 1);
}

std::pair<std::vector<cpplot2d::Plot2D::GuiLine>, std::vector<cpplot2d::Plot2D::GuiText>>
cpplot2d::Plot2D::GetPlotBorderTickLines(const WindowRect& rect, Color color)
{
    const int leftBorderPos = rect.left + (int)m_plotBorderOffsets.first;
    const int rightBorderPos = rect.right - (int)m_plotBorderOffsets.first;
    const int topBorderPos = rect.top - (int)m_plotBorderOffsets.second;
    const int bottomBorderPos = rect.bottom + (int)m_plotBorderOffsets.second;
    const Dimension2d charSize = m_charSize;
    std::vector<GuiLine> ticks;
    std::vector<GuiText> labels;
    std::stringstream label;

    // Draw X-axis ticks
    int numTicksX = m_plotProperties.tickLineCount;
    int x = 0;
    float offset = m_viewZero.first;
    int tickInterval = (rightBorderPos - leftBorderPos) / (numTicksX + 1);
    float increment = m_viewSpanY / ((float)numTicksX + 1);

    for (int i = 1; i <= numTicksX; ++i)
    {
        x = leftBorderPos + i * tickInterval;
        offset += increment;

        ticks.push_back(GuiLine({x, bottomBorderPos + m_tickLength},
                                {x, bottomBorderPos - m_tickLength}, color));

        label.str("");
        label << std::setprecision(3) << offset;
        labels.push_back(GuiText(
            label.str(), {x - charSize.first * 3, bottomBorderPos - m_tickLength - charSize.second},
            color, Orientation::HORIZONTAL));
    }
    ticks.push_back(GuiLine({rightBorderPos, bottomBorderPos + m_tickLength},
                            {rightBorderPos, bottomBorderPos - m_tickLength}, color));
    label.str("");
    label << std::setprecision(3) << offset + increment;
    labels.push_back(GuiText(
        label.str(),
        {rightBorderPos - charSize.first * 3, bottomBorderPos - m_tickLength - charSize.second},
        color, Orientation::HORIZONTAL));

    // Draw Y-axis ticks
    int numTicksY = m_plotProperties.tickLineCount;
    int y = 0;
    increment = m_viewSpanY / ((float)numTicksY + 1.f);
    offset = m_viewZero.second;
    tickInterval = (topBorderPos - bottomBorderPos) / (numTicksY + 1);

    for (int i = 1; i <= numTicksY; i++)
    {
        y = bottomBorderPos + (i * tickInterval);
        offset += increment;

        ticks.push_back(
            GuiLine({leftBorderPos - m_tickLength, y}, {leftBorderPos + m_tickLength, y}, color));

        label.str("");
        label << std::setprecision(3) << offset;
        labels.push_back(GuiText(label.str(),
                                 Point({std::max(10, leftBorderPos - charSize.first - m_tickLength),
                                        y - (int)(0.5 * charSize.second)}),
                                 color, Orientation::HORIZONTAL, 10, m_font, Alignment::RIGHT));
    }
    ticks.push_back(GuiLine({leftBorderPos - m_tickLength, topBorderPos},
                            {leftBorderPos + m_tickLength, topBorderPos}, color));
    label.str("");
    label << std::setprecision(3) << offset;
    labels.push_back(
        GuiText(label.str(),
                Point({std::max(10, int(leftBorderPos - charSize.first - m_tickLength)),
                       topBorderPos - (int)(0.5 * charSize.second)}),
                color, Orientation::HORIZONTAL, 10, m_font, Alignment::RIGHT));

    return {ticks, labels};
}

std::vector<cpplot2d::Plot2D::GuiText> cpplot2d::Plot2D::GetPlotLabels(const WindowRect& rect,
                                                                       Color color)
{
    const Dimension2d charSize = m_charSize;
    const std::string font = m_font;
    std::vector<GuiText> labels{3};
    labels[0] = GuiText(xLabel,
                        Point((int)(rect.right / 2) - (int)xLabel.size() * charSize.first,
                              rect.bottom + charSize.second),
                        color, Orientation::HORIZONTAL, 12, font);  // X-axis label
    labels[1] = GuiText(yLabel,
                        Point(rect.left + charSize.second,
                              (int)((rect.top - (int)yLabel.size() * charSize.first) / 2)),
                        color, Orientation::VERTICAL, 12, font);  // Y-axis label
    labels[2] = GuiText(title,
                        Point((int)(rect.right / 2) - (int)title.size() * charSize.first,
                              rect.top - (int)m_plotBorderOffsets.second + charSize.second),
                        color, Orientation::HORIZONTAL, 14, font);  // Plot title

    return labels;
}
inline void cpplot2d::Plot2D::UpdateViewportWindowScaleFactors(const Dimension2d& windowSize)
{
    Pointf viewportToWindow = GetViewportToWindowScaleFactor(windowSize);
    m_viewportToWindowScaleFactors = viewportToWindow;
    m_windowToViewportScaleFactors = {1 / viewportToWindow.first, 1 / viewportToWindow.second};
}

inline cpplot2d::detail::Pointf cpplot2d::Plot2D::GetViewportToWindowScaleFactor(
    const Dimension2d& windowSize)
{
    const float xPlotSpan = (float)windowSize.first - 2 * m_plotBorderOffsets.first;
    const float yPlotSpan = (float)windowSize.second - 2 * m_plotBorderOffsets.second;

    const float viewSpanX = (m_viewSpanX > 0.0f) ? m_viewSpanX : 1.0f;
    const float viewSpanY = (m_viewSpanY > 0.0f) ? m_viewSpanY : 1.0f;

    return {viewSpanX / xPlotSpan, viewSpanY / yPlotSpan};
}

void cpplot2d::Plot2D::GetDataPolyline(const LineSeries& series, const WindowRect& rect,
                                       GuiPolyline& output)
{
    if (series.size == 0) return;

    // Store member variables for faster access
    const WindowRect vRect = m_viewportRect;
    const size_t seriesSize = series.data.size();
    const Pointf windowToViewportScales = m_windowToViewportScaleFactors;
    const float xMul = windowToViewportScales.first;
    const float xAdd = m_plotBorderOffsets.first - m_viewZero.first * windowToViewportScales.first;
    const float yMul = windowToViewportScales.second;
    const float yAdd =
        m_plotBorderOffsets.second - m_viewZero.second * windowToViewportScales.second;

    m_polylineBuffer.clear();

    // Initialization
    Point transformedPoint{static_cast<int>(series.data[0].first * xMul + xAdd),
                           static_cast<int>(series.data[0].second * yMul + yAdd)};
    Point lastPoint = transformedPoint;
    bool lastPointInsideViewport = IsPointInsideRect(
        lastPoint, vRect);  // Cache to prevent checking bounds on 2 points every iteration.
    bool currentPointInsideViewport = lastPointInsideViewport;
    if (currentPointInsideViewport) m_polylineBuffer.emplace_back(transformedPoint);

    // State encodings to prevent nested ifs
    int lastIn = lastPointInsideViewport;
    int currIn;
    int state;

    for (int i = 1; i < seriesSize; i++)
    {
        transformedPoint.first = static_cast<int>(series.data[i].first * xMul + xAdd);
        transformedPoint.second = static_cast<int>(series.data[i].second * yMul + yAdd);

        // Get axis intercept for point pairs that span across plot boundaries
        currentPointInsideViewport = IsPointInsideRect(transformedPoint, vRect);
        currIn = currentPointInsideViewport;
        state = (lastIn << 1) | currIn;

        switch (state)
        {
            case 0b11:  // both inside
                if (lastPoint != transformedPoint) m_polylineBuffer.emplace_back(transformedPoint);
                break;

            case 0b10:  // leaving
                m_polylineBuffer.emplace_back(
                    GetIntersectionPointOnRect(lastPoint, transformedPoint, vRect));
                break;
            case 0b01:  // entering
                m_polylineBuffer.emplace_back(
                    GetIntersectionPointOnRect(transformedPoint, lastPoint, vRect));
                m_polylineBuffer.emplace_back(transformedPoint);

                break;

            case 0b00:  // both outside
            {
                if (lastPoint == transformedPoint)
                {
                    continue;
                }
                Point p1, p2;
                int count = GetIntersectionPointsOnRect(lastPoint, transformedPoint, vRect, p1, p2);
                switch (count)
                {
                    case 0:
                        break;
                    case 2:
                        // Emplace points in correct order (sort by distance to last point)
                        if (DistanceSquared(lastPoint, p2) > DistanceSquared(lastPoint, p1))
                        {
                            m_polylineBuffer.emplace_back(p1);
                            m_polylineBuffer.emplace_back(p2);
                        }
                        else
                        {
                            m_polylineBuffer.emplace_back(p2);
                            m_polylineBuffer.emplace_back(p1);
                        }
                        break;
                    default:
                        break;
                }

                break;
            }
            default:
                continue;
        }

        lastPoint = transformedPoint;
        lastIn = currentPointInsideViewport;
    }

    output.points = m_polylineBuffer;
    output.color = series.color;
}

inline int cpplot2d::Plot2D::DistanceSquared(const Point& p1, const Point& p2)
{
    return (p1.first - p2.first) * (p1.first - p2.first) +
           (p1.second - p2.second) * (p1.second - p2.second);
}

inline int cpplot2d::Plot2D::GetIntersectionPointsOnRect(const Point& p1, const Point& p2,
                                                         const WindowRect& rect, Point& out1,
                                                         Point& out2)
{
    int out = 0;
    Point temp;
    if (segmentsIntersect(p1, p2, {rect.left, rect.top}, {rect.right, rect.top}))
    {
        if (IntersectionPoint(p1, p2, {rect.left, rect.top}, {rect.right, rect.top}, out1))
        {
            out++;
        }
        else
        {
            // Colinear. return both original points
            out1 = p1;
            out2 = p2;
            return 2;
        }
    }
    if (segmentsIntersect(p1, p2, {rect.left, rect.top}, {rect.left, rect.bottom}))
    {
        if (IntersectionPoint(p1, p2, {rect.left, rect.top}, {rect.left, rect.bottom}, temp))
        {
            (out == 1) ? (out2 = temp) : (out1 = temp);
            out++;
            // Can have at most 2 intersection points, so this is the earliest that both points can
            // be set
            if (out == 2) return out;
        }
        else
        {
            out1 = p1;
            out2 = p2;
            return 2;
        }
    }
    if (segmentsIntersect(p1, p2, {rect.left, rect.bottom}, {rect.right, rect.bottom}))
    {
        if (IntersectionPoint(p1, p2, {rect.left, rect.bottom}, {rect.right, rect.bottom}, temp))
        {
            (out == 1) ? (out2 = temp) : (out1 = temp);
            out++;
            if (out == 2) return out;
        }
        else
        {
            out1 = p1;
            out2 = p2;
            return 2;
        }
    }
    if (segmentsIntersect(p1, p2, {rect.right, rect.bottom}, {rect.right, rect.top}))
    {
        if (IntersectionPoint(p1, p2, {rect.right, rect.bottom}, {rect.right, rect.top}, temp))
        {
            (out == 1) ? (out2 = temp) : (out1 = temp);
            out++;
        }
        else
        {
            out1 = p1;
            out2 = p2;
            return 2;
        }
    }

    return out;
}

inline cpplot2d::Plot2D::Point cpplot2d::Plot2D::GetIntersectionPointOnRect(const Point& p1,
                                                                            const Point& p2,
                                                                            const WindowRect& rect)
{
    Point p;
    if (segmentsIntersect(p1, p2, {rect.left, rect.top}, {rect.right, rect.top}))
    {
        IntersectionPoint(p1, p2, {rect.left, rect.top}, {rect.right, rect.top}, p);
        return p;
    }
    if (segmentsIntersect(p1, p2, {rect.left, rect.top}, {rect.left, rect.bottom}))
    {
        IntersectionPoint(p1, p2, {rect.left, rect.top}, {rect.left, rect.bottom}, p);
        return p;
    }
    if (segmentsIntersect(p1, p2, {rect.left, rect.bottom}, {rect.right, rect.bottom}))
    {
        IntersectionPoint(p1, p2, {rect.left, rect.bottom}, {rect.right, rect.bottom}, p);
        return p;
    }
    if (segmentsIntersect(p1, p2, {rect.right, rect.bottom}, {rect.right, rect.top}))
    {
        IntersectionPoint(p1, p2, {rect.right, rect.bottom}, {rect.right, rect.top}, p);
        return p;
    }

    return p2;
}

int64_t cpplot2d::Plot2D::Cross(const Point& a, const Point& b, const Point& c)
{
    return (int64_t)(b.first - a.first) * (c.second - a.second) -
           (int64_t)(b.second - a.second) * (c.first - a.first);
}
bool cpplot2d::Plot2D::onSegment(const Point& a, const Point& b, const Point& c)
{
    return std::min(a.first, b.first) <= c.first && c.first <= std::max(a.first, b.first) &&
           std::min(a.second, b.second) <= c.second && c.second <= std::max(a.second, b.second);
}
bool cpplot2d::Plot2D::segmentsIntersect(const Point& p1, const Point& p2, const Point& p3,
                                         const Point& p4)
{
    int64_t d1 = Cross(p1, p2, p3);
    int64_t d2 = Cross(p1, p2, p4);
    int64_t d3 = Cross(p3, p4, p1);
    int64_t d4 = Cross(p3, p4, p2);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        return true;

    if (d1 == 0 && onSegment(p1, p2, p3)) return true;
    if (d2 == 0 && onSegment(p1, p2, p4)) return true;
    if (d3 == 0 && onSegment(p3, p4, p1)) return true;
    if (d4 == 0 && onSegment(p3, p4, p2)) return true;

    return false;
}
bool cpplot2d::Plot2D::IntersectionPoint(const Point& a, const Point& b, const Point& c,
                                         const Point& d, Point& result)
{
    double x1 = a.first, y1 = a.second;
    double x2 = b.first, y2 = b.second;
    double x3 = c.first, y3 = c.second;
    double x4 = d.first, y4 = d.second;

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    if (denom == 0.0) return false;  // Parallel or collinear

    double px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;

    double py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

    // Check if intersection lies on both segments
    auto onSeg = [](double x, double y, const Point& p, const Point& q)
    {
        return x >= std::min(p.first, q.first) - 1e-9 && x <= std::max(p.first, q.first) + 1e-9 &&
               y >= std::min(p.second, q.second) - 1e-9 && y <= std::max(p.second, q.second) + 1e-9;
    };

    if (onSeg(px, py, a, b) && onSeg(px, py, c, d))
    {
        result = {(int)px, (int)py};
        return true;
    }
    // Colinear
    return false;
}
void cpplot2d::Plot2D::Show(bool block)
{
    m_window->SetIsVisible(true);
    if (block) m_window->RunEventLoop();
}
inline cpplot2d::Plot2D::Pointf cpplot2d::Plot2D::GetPlotBorderOffsets()
{
    return m_plotBorderOffsets;
}
void cpplot2d::Plot2D::SetPlotBorderOffsets(Pointf offsets)
{
    m_plotBorderOffsets = offsets;
}
void cpplot2d::Plot2D::SetViewportRect(const WindowRect& rect)
{
    m_viewportRect = rect;
}

void cpplot2d::Plot2D::OnWindowResizeCallback(IWindow& window)
{
    Dimension2d size = window.GetRect().Size();
    float verticalOffset = (float)size.second * m_plotBorderOffsetFactor;
    float horizontalOffset = (float)size.first * m_plotBorderOffsetFactor;
    SetPlotBorderOffsets(Pointf{horizontalOffset, verticalOffset});
    SetViewportRect(WindowRect(size.second - (int)verticalOffset, (int)horizontalOffset,
                               size.first - (int)horizontalOffset, (int)verticalOffset));

    UpdateViewportWindowScaleFactors(size);
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    window.Invalidate(m_plotWindowState.get());
}

cpplot2d::detail::Pointf cpplot2d::Plot2D::GetTransformedCoordinates(Point coord)
{
    const Pointf offsets = m_plotBorderOffsets;
    const Pointf viewportOrigin = m_viewZero;
    const Pointf scaleFactors = m_viewportToWindowScaleFactors;

    return {((float)coord.first - offsets.first) * scaleFactors.first + viewportOrigin.first,
            (((float)coord.second - offsets.second) * scaleFactors.second + viewportOrigin.second)};
}

void cpplot2d::Plot2D::OnMouseMoveCallback(IWindow& w, Point mousePos)
{
    switch (m_InteractionMode)
    {
        case InteractionMode::NONE:
            HandleMouseHover(w, mousePos);
            break;
        case InteractionMode::PAN_DEFAULT:
        case InteractionMode::ZOOM_DEFAULT:
            break;
        case InteractionMode::PAN_ACTIVE:
            HandlePanDrag(w, mousePos);
            break;
        case InteractionMode::ZOOM_ACTIVE:
            HandleZoomDrag(w, mousePos);
            break;
    }
}
void cpplot2d::Plot2D::HandlePanDrag(IWindow& w, Point mousePos)
{
    const Pointf scales = m_viewportToWindowScaleFactors;
    const Point lastMousePos = m_lastMousePos;

    // Calculate mouse movement delta in data coords
    // And update view offsets
    m_viewZero.first += static_cast<float>(lastMousePos.first - mousePos.first) * scales.first;
    m_viewZero.second += static_cast<float>(lastMousePos.second - mousePos.second) * scales.second;

    // Update and redraw
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    w.Invalidate(m_plotWindowState.get());

    m_lastMousePos = mousePos;
}
void cpplot2d::Plot2D::HandleZoomDrag(IWindow& w, Point mousePos)
{
    // Determine rectangle corners
    int x1 = std::clamp(std::min(m_lastMousePos.first, mousePos.first), m_viewportRect.left,
                        m_viewportRect.right);
    int x2 = std::clamp(std::max(m_lastMousePos.first, mousePos.first), m_viewportRect.left,
                        m_viewportRect.right);
    int y1 = std::clamp(std::min(m_lastMousePos.second, mousePos.second), m_viewportRect.bottom,
                        m_viewportRect.top);
    int y2 = std::clamp(std::max(m_lastMousePos.second, mousePos.second), m_viewportRect.bottom,
                        m_viewportRect.top);
    // Draw rectangle overlay
    m_plotWindowState->rects.back() = {(GuiRect({x1, y2}, {x2, y1}, Color::Yellow()))};
    w.Invalidate(m_plotWindowState.get());
}
inline const bool cpplot2d::Plot2D::IsPointInsideRect(const Point& p, const WindowRect& rect)
{
    return (p.first > rect.left && p.first < rect.right) &&
           (p.second > rect.bottom && p.second < rect.top);
}

void cpplot2d::Plot2D::HandleMouseHover(IWindow& w, Point mousePos)
{
    WindowRect rect = w.GetRect();

    // Mouse coordinates should fit inside a rect that starts at the top left corner and extends
    // 250 wide and 30 high
    WindowRect mouseCoordinateRect(rect.top, rect.left,
                                   rect.left + m_mouseCoordinateRectOffset.first,
                                   rect.top - m_mouseCoordinateRectOffset.second);

    if (IsPointInsideRect(mousePos, m_viewportRect))
    {
        auto transformedCoords = GetTransformedCoordinates({mousePos.first, mousePos.second});

        char buf[64];
        int n = snprintf(buf, sizeof(buf), "(X, Y) = (%.3g, %.3g)", transformedCoords.first,
                         transformedCoords.second);
        std::string coordText(buf, n > 0 ? n : 0);

        // Start mouse coordinate text at top left corner with small offset
        int x = rect.left + 1 * m_charSize.first;
        int y = rect.top - 1 - m_charSize.second;

        m_coordinateViewState->text = {
            GuiText(coordText, Point(x, y), Color::White(), Orientation::HORIZONTAL)};
        m_coordinateViewState->background = Color::Black();
        w.InvalidateRegion(mouseCoordinateRect, m_coordinateViewState.get());
        m_invalidateMouseCoordRegion = true;
    }
    else if (m_invalidateMouseCoordRegion)
    {
        // Invalidate only once when mouse leaves plot area
        m_invalidateMouseCoordRegion = false;

        // Invalidate whole plot window to clear mouse coordinate text
        // and make sure any overlapped elements are redrawn
        w.Invalidate(m_plotWindowState.get());
    }
}

void cpplot2d::Plot2D::OnMouseLButtonDownCallback(IWindow& w, Point mousePos)
{
    if (!IsPointInsideRect(mousePos, m_viewportRect))
    {
        return;
    }

    m_lastMousePos = mousePos;
    switch (m_InteractionMode)
    {
        case InteractionMode::NONE:
            break;
        case InteractionMode::PAN_DEFAULT:
            m_InteractionMode = InteractionMode::PAN_ACTIVE;
            break;
        case InteractionMode::ZOOM_DEFAULT:
            m_InteractionMode = InteractionMode::ZOOM_ACTIVE;
            break;
        default:
            break;
    }
}
void cpplot2d::Plot2D::OnMouseLButtonUpCallback(IWindow& w, Point mousePos)
{
    switch (m_InteractionMode)
    {
        case InteractionMode::NONE:
        case InteractionMode::PAN_DEFAULT:
        case InteractionMode::ZOOM_DEFAULT:
            break;
        case InteractionMode::PAN_ACTIVE:
            m_InteractionMode = InteractionMode::PAN_DEFAULT;
            break;
        case InteractionMode::ZOOM_ACTIVE:
            m_InteractionMode = InteractionMode::ZOOM_DEFAULT;
            WindowRect rect;
            rect.left = std::clamp(std::min(m_lastMousePos.first, mousePos.first),
                                   m_viewportRect.left, m_viewportRect.right);

            rect.right = std::clamp(std::max(m_lastMousePos.first, mousePos.first),
                                    m_viewportRect.left, m_viewportRect.right);

            rect.bottom = std::clamp(std::min(m_lastMousePos.second, mousePos.second),
                                     m_viewportRect.bottom, m_viewportRect.top);
            rect.top = std::clamp(std::max(m_lastMousePos.second, mousePos.second),
                                  m_viewportRect.bottom, m_viewportRect.top);
            Zoom(rect, w);
            break;
    }
}
void cpplot2d::Plot2D::Zoom(WindowRect zoomRect, IWindow& w)
{
    // Set plot view to match rectangle
    Dimension2d windowSize = w.GetRect().Size();
    const Pointf windowToViewportScales = m_windowToViewportScaleFactors;
    const Pointf viewportToWindowScales = m_viewportToWindowScaleFactors;
    UpdateViewportWindowScaleFactors(windowSize);
    const Pointf scales = m_viewportToWindowScaleFactors;

    m_viewZero.first =
        ((float)zoomRect.left - m_plotBorderOffsets.first) * scales.first + m_viewZero.first;
    m_viewZero.second =
        ((float)zoomRect.bottom - m_plotBorderOffsets.second) * scales.second + m_viewZero.second;
    m_viewSpanX = static_cast<float>(zoomRect.right - zoomRect.left) * scales.first;
    m_viewSpanY = static_cast<float>(zoomRect.top - zoomRect.bottom) * scales.second;

    // Update and redraw
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    w.Invalidate(m_plotWindowState.get());
}

void cpplot2d::Plot2D::OnSaveClicked(IWindow& w)
{
    w.SaveScreenshot(title + "_" + w.GetTimestamp());
}
void cpplot2d::Plot2D::OnToggleZoomClicked(IWindow& w)
{
    if (m_InteractionMode == InteractionMode::ZOOM_DEFAULT)
    {
        m_InteractionMode = InteractionMode::NONE;
        // Remove interaction mode text
        m_plotWindowState->text.back() = {};
    }
    else
    {
        m_InteractionMode = InteractionMode::ZOOM_DEFAULT;
        // Add interaction mode text
        WindowRect rect = w.GetRect();
        m_plotWindowState->text.back() = GetInteractionText("Zoom Mode Active", rect);
    }
    w.Invalidate(m_plotWindowState.get());
}
cpplot2d::detail::GuiText cpplot2d::Plot2D::GetInteractionText(const std::string& label,
                                                               WindowRect rect)
{
    return GuiText(label, Point(rect.left + 5, rect.top - m_charSize.second), Color::Green(),
                   Orientation::HORIZONTAL);
}

void cpplot2d::Plot2D::OnToggleGrabClicked(IWindow& w)
{
    if (m_InteractionMode == InteractionMode::PAN_DEFAULT)
    {
        m_InteractionMode = InteractionMode::NONE;
        m_plotWindowState->text.back() = {};  // Remove interaction mode text
    }
    else
    {
        m_InteractionMode = InteractionMode::PAN_DEFAULT;
        // Add interaction mode text
        WindowRect rect = w.GetRect();
        m_plotWindowState->text.back() = GetInteractionText("Grab Mode Active", rect);
    }
    w.Invalidate(m_plotWindowState.get());
}
void cpplot2d::Plot2D::OnResetViewClicked(IWindow& w)
{
    // Temporarily disable mouse hover to avoid blanking during next update
    m_window->OnMouseMoveCallback = nullptr;

    // Restore saved defaults
    m_viewZero = m_defaultViewZero;
    m_viewSpanX = m_defaultViewSpanX;
    m_viewSpanY = m_defaultViewSpanY;
    UpdateViewportWindowScaleFactors(w.GetRect().Size());
    UpdatePlotWindowState(m_plotWindowState.get(), *m_window);
    w.Invalidate(m_plotWindowState.get());

    // Re-enable mouse hover
    m_window->OnMouseMoveCallback = [this](Point p) { this->OnMouseMoveCallback(*m_window, p); };
}
std::string cpplot2d::Plot2D::FileName::Create(const std::string& dir, const std::string& filename)
{
    std::string safeFilename = filename;
    std::replace(safeFilename.begin(), safeFilename.end(), ':', '_');

#ifdef _WIN32
    std::string sep = "\\";
#else
    std::string sep = "/";
#endif

    std::string name = dir + sep + safeFilename;
    return name;
}

#ifdef CPPLOT2D_HEADLESS

void cpplot2d::Plot2D::NullGraphicsContext::Init()
{
}
void cpplot2d::Plot2D::NullGraphicsContext::Shutdown()
{
}
cpplot2d::Plot2D::IWindow* cpplot2d::Plot2D::NullGraphicsContext::MakeWindow(
    WindowState* initialState, Dimension2d defaultSize, Dimension2d minSize, bool isVisible,
    const std::string& title)
{
    return new cpplot2d::Plot2D::NullWindow();
}
void cpplot2d::Plot2D::NullWindow::Invalidate(WindowState* windowState)
{
}
cpplot2d::detail::Dimension2d cpplot2d::Plot2D::NullWindow::GetAverageCharSize()
{
    return {0, 0};
}
void cpplot2d::Plot2D::NullWindow::InvalidateRegion(const WindowRect& rect,
                                                    WindowState* windowState)
{
}
void cpplot2d::Plot2D::NullWindow::AddMenuButtons(const std::string menu, MenuButtons menuButtons)
{
}
bool cpplot2d::Plot2D::NullWindow::SaveScreenshot(const std::string& fileName)
{
    return true;
}
void cpplot2d::Plot2D::NullWindow::SetIsVisible(bool isVisible)
{
}
std::string cpplot2d::Plot2D::NullWindow::GetTimestamp()
{
    return "";
}
cpplot2d::Plot2D::WindowRect cpplot2d::Plot2D::NullWindow::GetRect()
{
    return WindowRect();
}
void cpplot2d::Plot2D::NullWindow::RunEventLoop()
{
}
#endif  // CPPLOT2D_HEADLESS

#ifdef _WIN32
void cpplot2d::Plot2D::Win32Window::RunEventLoop()
{
    // Standard Win32 blocking message loop exits when WindowProc posts PostQuitMessage
    MSG msg;
    while (GetMessage(&msg, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}

void cpplot2d::Plot2D::Win32GraphicsContext::Init()
{
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    Gdiplus::GdiplusStartup(&m_gdiplusToken, &gdiplusStartupInput, nullptr);
}

void cpplot2d::Plot2D::Win32GraphicsContext::Shutdown()
{
    Gdiplus::GdiplusShutdown(m_gdiplusToken);
}

cpplot2d::Plot2D::IWindow* cpplot2d::Plot2D::Win32GraphicsContext::MakeWindow(
    WindowState* initialState, Dimension2d defaultSize, Dimension2d minSize, bool isVisible,
    const std::string& title)
{
    return new cpplot2d::Plot2D::Win32Window(defaultSize, Dimension2d(0, 0), title, initialState);
}
cpplot2d::Plot2D::Win32Window::Win32Window(Dimension2d defaultWindowSize, Dimension2d pos,
                                           std::string title, WindowState* initialState)
{
    m_windowState = initialState;

    // Register window class
    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = TEXT("PlotWindowClass");
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

    if (!RegisterClass(&wc))
    {
        DWORD err = GetLastError();
        // If class already exists that's fine; treat ERROR_CLASS_ALREADY_EXISTS as non-fatal
        if (err != ERROR_CLASS_ALREADY_EXISTS)
        {
            DWORD err = GetLastError();
            std::ostringstream oss;
            oss << "Failed to create window. Error: " << err;
            CPPLOT2D_DEBUG("%s", oss.str().c_str());
        }
    }

    m_hwnd =
        CreateWindowEx(0, TEXT("PlotWindowClass"), title.c_str(), WS_OVERLAPPEDWINDOW,
                       CW_USEDEFAULT, CW_USEDEFAULT, defaultWindowSize.first,
                       defaultWindowSize.second, nullptr, nullptr, GetModuleHandle(nullptr), this);

    if (!m_hwnd)
    {
        DWORD err = GetLastError();
        std::ostringstream oss;
        oss << "Failed to create window. Error: " << err;
        throw std::runtime_error(oss.str());
    }

    m_hMenu = CreateMenu();

    ResizeBackBuffer(m_hwnd);

    HDC hdc = GetDC(m_hwnd);
    SetGraphicsMode(hdc, GM_ADVANCED);
    ReleaseDC(m_hwnd, hdc);
}

void cpplot2d::Plot2D::Win32Window::AddMenuButtons(const std::string menu, MenuButtons menuButtons)
{
    HMENU hNewMenu = CreateMenu();
    for (const auto& pair : menuButtons)
    {
        int id = IDGenerator::Next();
        AppendMenu(hNewMenu, MF_STRING, id, pair.first.c_str());
        m_menuCommands.emplace(id, pair.second);
    }

    AppendMenu(m_hMenu, MF_POPUP, (UINT_PTR)hNewMenu, menu.c_str());
    SetMenu(m_hwnd, m_hMenu);
}

std::string cpplot2d::Plot2D::Win32Window::GetTimestamp()
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
cpplot2d::Plot2D::WindowRect cpplot2d::Plot2D::Win32Window::GetRect()
{
    RECT rect;
    GetClientRect(m_hwnd, &rect);

    // Universal window assumes origin at bottom left. Win32 is at top left
    return WindowRect(rect.bottom, rect.left, rect.right, rect.top);
}

void cpplot2d::Plot2D::Win32Window::SetIsVisible(bool isVisible)
{
    ShowWindow(m_hwnd, isVisible);
}

bool cpplot2d::Plot2D::Win32Window::SaveScreenshot(const std::string& fileName)
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

HBITMAP cpplot2d::Plot2D::Win32Window::CaptureWindowContent(HWND hwnd)
{
    RECT rect;
    if (!GetClientRect(hwnd, &rect)) return nullptr;
    int width = rect.right - rect.left;
    int height = rect.bottom - rect.top;
    if (width <= 0 || height <= 0) return nullptr;

    HDC hdcWindow = GetDC(hwnd);
    if (!hdcWindow) return nullptr;

    HDC hdcMemDC = CreateCompatibleDC(hdcWindow);
    if (!hdcMemDC)
    {
        ReleaseDC(hwnd, hdcWindow);
        return nullptr;
    }

    HBITMAP hbmScreen = CreateCompatibleBitmap(hdcWindow, width, height);
    if (!hbmScreen)
    {
        DeleteDC(hdcMemDC);
        ReleaseDC(hwnd, hdcWindow);
        return nullptr;
    }

    // Select the bitmap into the mem DC and save the old one
    HBITMAP hOldBmp = (HBITMAP)SelectObject(hdcMemDC, hbmScreen);

    BOOL success = FALSE;
    success = BitBlt(hdcMemDC, 0, 0, width, height, hdcWindow, 0, 0, SRCCOPY);

    // Restore original bitmap into mem DC before deleting DC
    SelectObject(hdcMemDC, hOldBmp);

    DeleteDC(hdcMemDC);
    ReleaseDC(hwnd, hdcWindow);

    if (!success)
    {
        // Clean up the bitmap
        DeleteObject(hbmScreen);
        return nullptr;
    }

    return hbmScreen;
}
void cpplot2d::Plot2D::Win32Window::SaveHBITMAPToFile(HBITMAP hBitmap, const std::string& filename)
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

    Gdiplus::Bitmap bitmap(hBitmap, nullptr);
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
    Gdiplus::Status status = bitmap.Save(wstr.c_str(), &clsid, nullptr);
    if (status != Gdiplus::Ok)
    {
        std::string msg = "Failed to save image: " + filename;
        MessageBox(m_hwnd, msg.c_str(), "Save Error", MB_OK | MB_ICONERROR);
    }
}
LRESULT CALLBACK cpplot2d::Plot2D::Win32Window::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam,
                                                           LPARAM lParam)
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

bool cpplot2d::Plot2D::Win32Window::BrowseForFolder(std::string& outFolder)
{
    BROWSEINFO bi;
    ZeroMemory(&bi, sizeof(bi));
    char szDisplayName[MAX_PATH];
    char szPath[MAX_PATH];

    bi.hwndOwner = nullptr;
    bi.pidlRoot = nullptr;
    bi.pszDisplayName = szDisplayName;
    bi.lpszTitle = "Choose Destination Folder";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    bi.lpfn = nullptr;
    bi.lParam = 0;
    bi.iImage = 0;

    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    if (pidl != nullptr)
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
cpplot2d::Plot2D::Dimension2d cpplot2d::Plot2D::Win32Window::GetAverageCharSize()
{
    HDC hdc = GetDC(m_hwnd);
    HFONT hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    HFONT old = (HFONT)SelectObject(hdc, hFont);
    TEXTMETRIC tm;
    GetTextMetrics(hdc, &tm);
    SelectObject(hdc, old);
    ReleaseDC(m_hwnd, hdc);

    return {tm.tmAveCharWidth, tm.tmHeight};
}

inline LRESULT cpplot2d::Plot2D::Win32Window::HandleMessage(HWND hwnd, UINT uMsg, WPARAM wParam,
                                                            LPARAM lParam)
{
    RECT rect;
    GetClientRect(hwnd, &rect);

    switch (uMsg)
    {
        case WM_PAINT:
        {
            if (!m_useCachedBitmap)
            {
                DrawWindowState(rect, m_invalidatedRegion);
            }
            else
            {
                DrawBitmap();
            }

            return 0;
        }
        case WM_SYSCOMMAND:
        {
            switch (wParam & 0xFFF0)
            {
                case SC_MOVE:
                    m_inMove = true;
                    m_inSize = false;
                    break;

                case SC_SIZE:
                    m_inSize = true;
                    m_inMove = false;
                    break;
            }
            break;
        }
        case WM_ENTERSIZEMOVE:
        {
            if (OnResizeStartCallback) OnResizeStartCallback();
            if (m_inMove)
            {
                m_useCachedBitmap = true;  // ONLY for dragging
            }
            return 0;
        }
        case WM_EXITSIZEMOVE:
        {
            m_inMove = m_inSize = false;
            m_useCachedBitmap = false;  // ONLY for dragging
            if (OnResizeEndCallback) OnResizeEndCallback();
            // InvalidateRect(hwnd, nullptr, TRUE);
            return 0;
        }
        case WM_SIZE:
        {
            ResizeBackBuffer(hwnd);
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
            std::pair<int, int> sizes = m_windowState->minSize;
            MINMAXINFO* minMaxInfo = (MINMAXINFO*)lParam;
            minMaxInfo->ptMinTrackSize.x = sizes.first;
            minMaxInfo->ptMinTrackSize.y = sizes.second;
            break;
        }
        case WM_LBUTTONDOWN:
        {
            // Get the mouse position
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);
            if (OnMouseLButtonDownCallback)
                OnMouseLButtonDownCallback(Point(
                    x, rect.bottom - y));  // Give Y in the expected universal coordinate system

            break;
        }
        case WM_LBUTTONUP:
        {
            // Get the mouse position
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);
            if (OnMouseLButtonUpCallback)
                OnMouseLButtonUpCallback(Point(
                    x, rect.bottom - y));  // Give Y in the expected universal coordinate system

            break;
        }
        case WM_SETCURSOR:
        {
            if (LOWORD(lParam) == HTCLIENT)
            {
                SetCursor(LoadCursor(nullptr, IDC_ARROW));
                return TRUE;
            }
            else
            {
                return DefWindowProc(hwnd, uMsg, wParam, lParam);
            }
        }
        case WM_SHOWWINDOW:
            ResizeBackBuffer(hwnd);
            if (OnResizeCallback) OnResizeCallback();
            return 0;
        case WM_MOUSEMOVE:
        {
            // Mouse move will trigger if window shows initially with the cursor over it.
            // Only allow mouse move events after the plot has been drawn at least once fully.
            if (!m_drawnOnce) return 0;

            // Get the mouse position
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);
            if (OnMouseMoveCallback)
                OnMouseMoveCallback(Point(
                    x, rect.bottom - y));  // Give Y in the expected universal coordinate system

            return 0;
        }
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
inline COLORREF cpplot2d::Plot2D::Win32Window::ToWin32Color(const Color color)
{
    return RGB(color.r, color.g, color.b);
}

void cpplot2d::Plot2D::Win32Window::DoDrawText(HDC hdc, GuiText text, RECT clientRect)
{
    SetGraphicsMode(hdc, GM_ADVANCED);
    HFONT oldFont = nullptr;
    HFONT font = nullptr;
    int height = GetTextHeight(hdc, text.size);
    if (text.orientation != Orientation::VERTICAL)
    {
        font = CreateFontOfSize(height, text.font);
    }
    else
    {
        font = CreateVerticalFont(height, text.font);
    }

    if (text.alignment != Alignment::RIGHT)
    {
        SetTextAlign(hdc, TA_LEFT | TA_TOP);
    }
    else
    {
        SetTextAlign(hdc, TA_RIGHT | TA_TOP);
    }
    oldFont = (HFONT)SelectObject(hdc, font);
    SetTextColor(hdc, ToWin32Color(text.color));
    SetBkMode(hdc, TRANSPARENT);
    TextOutA(hdc, text.pos.first, clientRect.bottom - text.pos.second + height, text.text.c_str(),
             static_cast<int>(text.text.length()));
    SelectObject(hdc, oldFont);
    DeleteObject(font);
}
HFONT cpplot2d::Plot2D::Win32Window::CreateVerticalFont(int height, const std::string& font)
{
    LOGFONTA lf = {};
    lf.lfHeight = height;
    lf.lfEscapement = 900;   // 90 degrees
    lf.lfOrientation = 900;  // 90 degrees
    lf.lfCharSet = ANSI_CHARSET;
    strcpy_s(lf.lfFaceName, font.c_str());

    return CreateFontIndirectA(&lf);
}
int cpplot2d::Plot2D::Win32Window::GetTextHeight(HDC hdc, int pointSize)
{
    int dpi = GetDeviceCaps(hdc, LOGPIXELSY);
    return -MulDiv(pointSize, dpi, 72);
}
HFONT cpplot2d::Plot2D::Win32Window::CreateFontOfSize(int height, const std::string& font)
{
    return CreateFontA(height, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, ANSI_CHARSET,
                       OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
                       DEFAULT_PITCH | FF_DONTCARE, font.c_str());
}

void cpplot2d::Plot2D::Win32Window::ResizeBackBuffer(HWND hwnd)
{
    HDC windowDC = GetDC(hwnd);

    RECT rc;
    GetClientRect(hwnd, &rc);
    int w = rc.right - rc.left;
    int h = rc.bottom - rc.top;

    if (m_backBuffer.backBuffer)
    {
        DeleteObject(m_backBuffer.backBuffer);
        m_backBuffer.backBuffer = nullptr;
    }
    if (m_backBuffer.backDC)
    {
        DeleteDC(m_backBuffer.backDC);
        m_backBuffer.backDC = nullptr;
    }

    m_backBuffer.backDC = CreateCompatibleDC(windowDC);
    m_backBuffer.backBuffer = CreateCompatibleBitmap(windowDC, w, h);

    SelectObject(m_backBuffer.backDC, m_backBuffer.backBuffer);

    m_backBuffer.bufferW = w;
    m_backBuffer.bufferH = h;

    ReleaseDC(hwnd, windowDC);
}
void cpplot2d::Plot2D::Win32Window::DrawBitmap()
{
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(m_hwnd, &ps);
    BitBlt(hdc, 0, 0, m_backBuffer.bufferW, m_backBuffer.bufferH, m_backBuffer.backDC, 0, 0,
           SRCCOPY);

    EndPaint(m_hwnd, &ps);
}

void cpplot2d::Plot2D::Win32Window::DrawWindowState(const RECT& clientRect,
                                                    const RECT& invalidatedRect)
{
    m_drawnOnce = true;
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(m_hwnd, &ps);
    SetGraphicsMode(hdc, GM_ADVANCED);
    // Fill background
    HBRUSH backgroundBrush = CreateSolidBrush(ToWin32Color(m_windowState->background));
    FillRect(m_backBuffer.backDC, &invalidatedRect, backgroundBrush);
    DeleteObject(backgroundBrush);

    // Draw Polylines
    std::map<Color, HPEN> brushes;  // HPEN map
    HPEN hpen = nullptr;
    std::map<Color, HPEN>::iterator it;
    for (const GuiPolyline& polyline : m_windowState->polylines)
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

        size_t size = polyline.points.size();
        if (size <= 0) continue;

        SelectObject(m_backBuffer.backDC, hpen);

        // Draw initial point, then LineTo the rest
        MoveToEx(m_backBuffer.backDC, polyline.points[0].first,
                 clientRect.bottom - polyline.points[0].second, nullptr);
        LineTo(m_backBuffer.backDC, polyline.points[0].first,
               clientRect.bottom - polyline.points[0].second);
        for (int i = 1; i < size; i++)
        {
            LineTo(m_backBuffer.backDC, polyline.points[i].first,
                   clientRect.bottom - polyline.points[i].second);
        }
    }
    // Draw Rects
    for (const GuiRect& guiRect : m_windowState->rects)
    {
        // Draw rectangle frame
        HBRUSH brush = CreateSolidBrush(ToWin32Color(guiRect.borderColor));
        // Add 1 to right side of rect since Win32 stops the rect 1 pixel prior
        const RECT winRect = {guiRect.topLeft.first, clientRect.bottom - guiRect.topLeft.second,
                              guiRect.bottomRight.first + 1,
                              clientRect.bottom - guiRect.bottomRight.second};
        FrameRect(m_backBuffer.backDC, &winRect, brush);
        DeleteObject(brush);
    }

    // Draw Lines
    for (const GuiLine& line : m_windowState->lines)
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

        SelectObject(m_backBuffer.backDC, hpen);

        // Draw initial point, then LineTo the rest
        MoveToEx(m_backBuffer.backDC, line.p1.first, clientRect.bottom - line.p1.second, nullptr);
        LineTo(m_backBuffer.backDC, line.p2.first, clientRect.bottom - line.p2.second);
    }

    // Draw Circles
    for (const GuiCircle& cirlce : m_windowState->circles)
    {
        HBRUSH brush = CreateSolidBrush(ToWin32Color(cirlce.fillColor));
        HBRUSH oldBrush = (HBRUSH)SelectObject(m_backBuffer.backDC, brush);
        Ellipse(m_backBuffer.backDC, cirlce.center.first - cirlce.radius,
                clientRect.bottom - (cirlce.center.second + cirlce.radius),
                cirlce.center.first + cirlce.radius,
                clientRect.bottom - (cirlce.center.second - cirlce.radius));
        SelectObject(m_backBuffer.backDC, oldBrush);
        DeleteObject(brush);
    }

    // Draw Text
    for (const GuiText& text : m_windowState->text)
    {
        DoDrawText(m_backBuffer.backDC, text, clientRect);
    }

    // Delete HPEN objects
    for (const auto& pair : brushes)
    {
        if (pair.second) DeleteObject(pair.second);
    }

    // Double-buffering: copy back buffer to window DC
    BitBlt(hdc, 0, 0, m_backBuffer.bufferW, m_backBuffer.bufferH, m_backBuffer.backDC, 0, 0,
           SRCCOPY);

    EndPaint(m_hwnd, &ps);
}

void cpplot2d::Plot2D::Win32Window::InvalidateRegion(const cpplot2d::Plot2D::WindowRect& windowRect,
                                                     cpplot2d::Plot2D::WindowState* windowState)
{
    m_windowState = windowState;
    RECT win32Rect;
    GetClientRect(m_hwnd, &win32Rect);

    // Universal coords assume origin at bottom left and extend up.
    // Win32 is the opposite, so reverse the Y coords.
    m_invalidatedRegion = {windowRect.left, win32Rect.bottom - windowRect.top, windowRect.right,
                           win32Rect.bottom - windowRect.bottom};
    InvalidateRect(m_hwnd, &m_invalidatedRegion, FALSE);
}

void cpplot2d::Plot2D::Win32Window::Invalidate(cpplot2d::Plot2D::WindowState* windowState)
{
    m_windowState = windowState;
    GetClientRect(m_hwnd, &m_invalidatedRegion);
    InvalidateRect(m_hwnd, nullptr, FALSE);
}
#endif  // _WIN32

#ifdef __APPLE__
#ifdef __OBJC__
// NOLINTBEGIN(readability-*, modernize-*, bugprone-*)
void Plot2D::CocoaGraphicsContext::Init()
{
    if (![NSApplication sharedApplication])
    {
        [NSApplication sharedApplication];
    }
}
void Plot2D::CocoaGraphicsContext::Shutdown()
{
    // No specific shutdown needed for Cocoa
}
cpplot2d::Plot2D::IWindow* Plot2D::CocoaGraphicsContext::MakeWindow(WindowState* windowState,
                                                                    Dimension2d defaultSize,
                                                                    Dimension2d minSize,
                                                                    bool isVisible,
                                                                    const std::string& title)
{
    auto window = new Plot2D::CocoaWindow(defaultSize, Dimension2d(0, 0), title, windowState);
    NSApplication* app = [NSApplication sharedApplication];
    [app finishLaunching];  // Ensure app is fully launched
    return window;
}

Plot2D::CocoaWindow::CocoaWindow(Dimension2d defaultWindowSize, Dimension2d pos, std::string title,
                                 WindowState* initialState)
{
    NSRect frame = NSMakeRect(0, 0, defaultWindowSize.first, defaultWindowSize.second);
    NSWindow* window =
        [[NSWindow alloc] initWithContentRect:frame
                                    styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                                               NSWindowStyleMaskResizable)
                                      backing:NSBackingStoreBuffered
                                        defer:NO];

    m_nsWindow = (void*)window;

    [window setTitle:[NSString stringWithUTF8String:title.c_str()]];
    [window makeKeyAndOrderFront:nil];
    [window setAcceptsMouseMovedEvents:YES];

    // Create and attach a delegate to handle window close
    WindowDelegate* delegate = [[WindowDelegate alloc] init];
    [window setDelegate:delegate];

    NSMenu* mainMenu = [[NSMenu alloc] init];
    m_mainMenu = (void*)mainMenu;

    NSMenuItem* appMenuItem = [NSMenuItem new];
    [mainMenu addItem:appMenuItem];
    NSMenu* appMenu = [[NSMenu alloc] init];
    [appMenuItem setSubmenu:appMenu];

    // Set the main menu
    [NSApp setMainMenu:mainMenu];

    // Create the WindowView instance and set it as the window's content view
    WindowView* windowView = [[WindowView alloc] initWithFrame:frame];
    [windowView setNeedsDisplay:YES];
    [window setContentView:windowView];
    [windowView display];

    m_windowView = (void*)windowView;

    // Subscribe to resize event
    id resizeObserver =
        [[NSNotificationCenter defaultCenter] addObserverForName:NSWindowDidResizeNotification
                                                          object:nil
                                                           queue:[NSOperationQueue mainQueue]
                                                      usingBlock:^(NSNotification* note) {
                                                        NSSize size = windowView.frame.size;
                                                        this->OnResize(size.width, size.height);
                                                      }];

    m_resizeObserver = (void*)resizeObserver;

    // Subscribe to mouse events
    // Subscribe to mouse move events
    id mouseMoveObserver =
        [[NSNotificationCenter defaultCenter] addObserverForName:@"MouseMovedNotification"
                                                          object:nil
                                                           queue:[NSOperationQueue mainQueue]
                                                      usingBlock:^(NSNotification* note) {
                                                        NSDictionary* userInfo = note.userInfo;
                                                        float x = [userInfo[@"x"] floatValue];
                                                        float y = [userInfo[@"y"] floatValue];
                                                        this->OnMouseMove(x, y);
                                                      }];
    m_mouseMoveObserver = (void*)mouseMoveObserver;

    // Subscribe to mouse down events
    id mouseDownObserver =
        [[NSNotificationCenter defaultCenter] addObserverForName:@"MouseDownNotification"
                                                          object:nil
                                                           queue:[NSOperationQueue mainQueue]
                                                      usingBlock:^(NSNotification* note) {
                                                        NSDictionary* userInfo = note.userInfo;
                                                        float x = [userInfo[@"x"] floatValue];
                                                        float y = [userInfo[@"y"] floatValue];
                                                        this->OnMouseLButtonDown(x, y);
                                                      }];
    m_mouseDownObserver = (void*)mouseDownObserver;

    // Subscribe to mouse up events
    id mouseUpObserver =
        [[NSNotificationCenter defaultCenter] addObserverForName:@"MouseUpNotification"
                                                          object:nil
                                                           queue:[NSOperationQueue mainQueue]
                                                      usingBlock:^(NSNotification* note) {
                                                        NSDictionary* userInfo = note.userInfo;
                                                        float x = [userInfo[@"x"] floatValue];
                                                        float y = [userInfo[@"y"] floatValue];
                                                        this->OnMouseLButtonUp(x, y);
                                                      }];
    m_mouseUpObserver = (void*)mouseUpObserver;
}
inline void Plot2D::CocoaWindow::OnResize(int width, int height)
{
    if (OnResizeCallback)
    {
        OnResizeCallback();
    }
}
inline void Plot2D::CocoaWindow::OnMouseMove(int x, int y)
{
    if (OnMouseMoveCallback)
    {
        OnMouseMoveCallback(Point(x, y));
    }
}

inline void Plot2D::CocoaWindow::OnMouseLButtonDown(int x, int y)
{
    if (OnMouseLButtonDownCallback)
    {
        OnMouseLButtonDownCallback(Point(x, y));
    }
}

inline void Plot2D::CocoaWindow::OnMouseLButtonUp(int x, int y)
{
    if (OnMouseLButtonUpCallback)
    {
        OnMouseLButtonUpCallback(Point(x, y));
    }
}

inline Plot2D::CocoaWindow::~CocoaWindow()
{
    for (void* bridge : m_menuCallbacks)
    {
        MenuCallbackBridge* mbridge = (MenuCallbackBridge*)bridge;
        [mbridge release];
    }
    m_menuCallbacks.clear();

    NSWindow* window = reinterpret_cast<NSWindow*>(m_nsWindow);
    [window release];

    NSMenu* mainMenu = reinterpret_cast<NSMenu*>(m_mainMenu);
    [mainMenu release];

    id resizeObserver = reinterpret_cast<id>(m_resizeObserver);
    id mouseMoveObserver = reinterpret_cast<id>(m_mouseMoveObserver);
    id mouseDownObserver = reinterpret_cast<id>(m_mouseDownObserver);
    id mouseUpObserver = reinterpret_cast<id>(m_mouseUpObserver);

    [[NSNotificationCenter defaultCenter] removeObserver:mouseUpObserver];
    [[NSNotificationCenter defaultCenter] removeObserver:mouseDownObserver];
    [[NSNotificationCenter defaultCenter] removeObserver:resizeObserver];
    [[NSNotificationCenter defaultCenter] removeObserver:mouseMoveObserver];
}

cpplot2d::Plot2D::Dimension2d Plot2D::CocoaWindow::GetAverageCharSize()
{
    NSFont* font = [NSFont systemFontOfSize:[NSFont systemFontSize]];
    NSDictionary* attributes = @{NSFontAttributeName : font};
    NSString* sampleText = @"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    NSSize textSize = [sampleText sizeWithAttributes:attributes];
    return {textSize.width / sampleText.length, textSize.height};
}

void Plot2D::CocoaWindow::AddMenuButtons(const std::string menu, MenuButtons menuButtons)
{
    NSString* menuTitle = [NSString stringWithUTF8String:menu.c_str()];
    NSMenu* mainMenu = reinterpret_cast<NSMenu*>(m_mainMenu);

    // Create the top-level menu item
    NSMenuItem* menuItem =
        [[NSMenuItem alloc] initWithTitle:menuTitle action:nil keyEquivalent:@""];
    [mainMenu addItem:menuItem];

    NSMenu* submenu = [[NSMenu alloc] initWithTitle:menuTitle];
    [menuItem setSubmenu:submenu];

    // Add buttons to the menu
    for (const auto& pair : menuButtons)
    {
        NSString* itemTitle = [NSString stringWithUTF8String:pair.first.c_str()];
        NSMenuItem* subItem = [[NSMenuItem alloc] initWithTitle:itemTitle
                                                         action:@selector(invokeCallback:)
                                                  keyEquivalent:@""];

        // Create a MenuCallbackBridge and store it as void*
        MenuCallbackBridge* bridge = [[MenuCallbackBridge alloc] initWithCallback:pair.second];
        m_menuCallbacks.push_back(reinterpret_cast<void*>(bridge));

        // Set the bridge as the target
        [subItem setTarget:bridge];
        [submenu addItem:subItem];
    }
}
void Plot2D::CocoaWindow::SetIsVisible(bool isVisible)
{
    NSWindow* window = reinterpret_cast<NSWindow*>(m_nsWindow);
    if (isVisible)
    {
        [window makeKeyAndOrderFront:nil];
        OnResizeCallback();
    }
    else
    {
        [window orderOut:nil];
    }
}
void Plot2D::CocoaWindow::Invalidate(WindowState* windowState)
{
    m_windowState = windowState;

    // Update the WindowView's pointer to the WindowState
    WindowView* view = reinterpret_cast<WindowView*>(m_windowView);
    view.windowState = windowState;

    // Mark the entire view for redraw
    [view setNeedsDisplay:YES];
}

std::string Plot2D::CocoaWindow::GetTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::string timestamp;

    std::tm local_time;
    localtime_r(&now_time, &local_time);
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y%m%d%H%M%S");
    timestamp = oss.str();

    return timestamp;
}
void Plot2D::CocoaWindow::RunEventLoop()
{
    // Start the event loop
    [NSApp run];
}
void Plot2D::CocoaWindow::InvalidateRegion(const WindowRect& rect, WindowState* windowState)
{
    m_windowState = windowState;

    // Update the WindowView's pointer to the WindowState
    WindowView* view = reinterpret_cast<WindowView*>(m_windowView);
    view.windowState = windowState;

    // Convert WindowRect (bottom-left origin) to NSRect (top-left origin)
    NSWindow* window = reinterpret_cast<NSWindow*>(m_nsWindow);
    NSRect contentRect = [window contentRectForFrameRect:[window frame]];

    NSRect nsRect = NSMakeRect(rect.left,               // x origin
                               rect.bottom,             // y origin
                               rect.right - rect.left,  // width
                               rect.top - rect.bottom   // height
    );

    // Mark the region for redraw
    [view setNeedsDisplayInRect:nsRect];
}
Plot2D::WindowRect Plot2D::CocoaWindow::GetRect()
{
    NSWindow* window = reinterpret_cast<NSWindow*>(m_nsWindow);
    NSRect contentRect = [window contentRectForFrameRect:[window frame]];
    return WindowRect(contentRect.size.height, 0, contentRect.size.width, 0);
}
bool Plot2D::CocoaWindow::SaveScreenshot(const std::string& fileName)
{
    WindowView* view = reinterpret_cast<WindowView*>(m_windowView);
    [view SaveScreenshot:fileName];
    return true;
}
// NOLINTEND(readability-*, modernize-*, bugprone-*)
#endif  // __OBJC__
#endif  //__APPLE__

#ifdef __linux__
void cpplot2d::Plot2D::X11GraphicsContext::Init()
{
}

void cpplot2d::Plot2D::X11GraphicsContext::Shutdown()
{
}
cpplot2d::Plot2D::IWindow* cpplot2d::Plot2D::X11GraphicsContext::MakeWindow(
    WindowState* initialState, Dimension2d defaultSize, Dimension2d minSize, bool isVisible,
    const std::string& title)
{
    return new X11Window(defaultSize, Dimension2d(0, 0), title, initialState);
}

cpplot2d::Plot2D::X11Window::X11Window(Dimension2d defaultWindowSize, Dimension2d pos,
                                       std::string title, WindowState* initialState)
{
    m_display = XOpenDisplay(nullptr);
    if (!m_display)
    {
        throw std::runtime_error("Failed to open X display.");
    }
    m_screen = DefaultScreen(m_display);
    Window root = RootWindow(m_display, m_screen);
    m_window = XCreateSimpleWindow(
        m_display, root, pos.first, pos.second,  // x, y (window position)
        defaultWindowSize.first,
        defaultWindowSize.second,  // window size
        1,                         // border width
        ToX11Pixel(initialState->background), ToX11Pixel(initialState->background));

    XSetWindowAttributes attrs;
    // Pin the current contents to the top-left (0,0) during resize
    attrs.bit_gravity = StaticGravity;
    attrs.background_pixmap = None;
    XChangeWindowAttributes(m_display, m_window, CWBackPixmap | CWBitGravity, &attrs);
    XSelectInput(m_display, m_window,
                 ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask |
                     ButtonReleaseMask | PointerMotionMask | StructureNotifyMask);
    m_windowDimensions = defaultWindowSize;

    m_wmDelete = XInternAtom(m_display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(m_display, m_window, &m_wmDelete, 1);

    XStoreName(m_display, m_window, title.c_str());

    // Create graphics context
    m_gc = XCreateGC(m_display, m_window, 0, nullptr);
    XSetLineAttributes(m_display, m_gc,
                       1,  // line width
                       LineSolid, CapButt, JoinMiter);

    // Load menu font
    if (!LoadFont())
    {
        throw std::runtime_error("Failed to load font for menu.");
    }
}
cpplot2d::Plot2D::X11Window::~X11Window()
{
    for (Menu& menu : m_menus)
    {
        if (menu.dropdown)
        {
            XDestroyWindow(m_display, menu.dropdown->win);
            delete menu.dropdown;
        }
    }
    auto it = m_sizeToFontCache.begin();
    auto end = m_sizeToFontCache.end();
    for (; it != end; ++it)
    {
        if (it->second) XFreeFont(m_display, it->second);
    }
    m_sizeToFontCache.clear();
    if (m_gc) XFreeGC(m_display, m_gc);
    if (m_backBuffer) XFreePixmap(m_display, m_backBuffer);
    XDestroyWindow(m_display, m_window);
    XCloseDisplay(m_display);
}
cpplot2d::Plot2D::Dimension2d cpplot2d::Plot2D::X11Window::GetAverageCharSize()
{
    XFontStruct* font = XLoadQueryFont(m_display, "fixed");
    if (!font)
    {
        throw std::runtime_error("Failed to load font 'fixed'.");
    }
    return {font->max_bounds.width, font->ascent + font->descent};
}
void cpplot2d::Plot2D::X11Window::AddMenuButtons(const std::string menu, MenuButtons menuButtons)
{
    m_menus.push_back(CreateMenu(menu, menuButtons));
}
cpplot2d::Plot2D::X11Window::Menu cpplot2d::Plot2D::X11Window::CreateMenu(
    const std::string& title, const MenuButtons& menuButtons)
{
    int x = 4;  // Start with some padding
    Menu menu;
    menu.title = title;
    int textWidth =
        XTextWidth(m_menuFont, menu.title.c_str(), static_cast<int>(menu.title.length()));
    menu.bounds = {0, x, x + textWidth + 16, m_menuBarHeight};

    const int itemHeight = 20;
    const int itemWidth = 100;

    XSetWindowAttributes attrs;
    attrs.override_redirect = True;  // No borders/decorations
    attrs.background_pixel = ToX11Pixel(m_menuColor);
    attrs.event_mask = ExposureMask | ButtonPressMask | PointerMotionMask;

    DropdownMenu* dropdownMenu = new DropdownMenu();
    dropdownMenu->x = menu.bounds.left;
    dropdownMenu->y = m_menuBarHeight;  // Below menu bar
    dropdownMenu->width = 100;          // Default width
    dropdownMenu->height = itemHeight * static_cast<int>(menuButtons.size());
    dropdownMenu->win = XCreateWindow(
        m_display, m_window, 0, 0, dropdownMenu->width, dropdownMenu->height, 1, CopyFromParent,
        InputOutput, CopyFromParent, CWOverrideRedirect | CWBackPixel | CWEventMask, &attrs);

    int y = 0;
    for (const auto& pair : menuButtons)
    {
        DropdownMenuItem item;
        item.text = pair.first;
        item.callback = pair.second;
        item.bounds = {y, menu.bounds.left, dropdownMenu->width, y + itemHeight};
        dropdownMenu->items.push_back(item);
        y += itemHeight;
    }

    menu.dropdown = dropdownMenu;

    return menu;
}
void cpplot2d::Plot2D::X11Window::DrawDropdownMenu(DropdownMenu* dropdownMenu)
{
    // Clear dropdown background
    XSetForeground(m_display, m_gc, ToX11Pixel(m_menuColor));
    XFillRectangle(m_display, dropdownMenu->win, m_gc, 0, 0, dropdownMenu->width,
                   dropdownMenu->height);

    XSetForeground(m_display, m_gc, ToX11Pixel(Color{0, 0, 0}));
    XSetFont(m_display, m_gc, m_menuFont->fid);

    // Draw menu items onto the dropdown window
    for (const DropdownMenuItem& item : dropdownMenu->items)
    {
        XDrawString(m_display, dropdownMenu->win, m_gc, item.bounds.left, item.bounds.bottom - 4,
                    item.text.c_str(), static_cast<int>(item.text.length()));
    }
}
void cpplot2d::Plot2D::X11Window::SetIsVisible(bool isVisible)
{
    if (isVisible)
        XMapWindow(m_display, m_window);
    else
        XUnmapWindow(m_display, m_window);

    XFlush(m_display);
}
void cpplot2d::Plot2D::X11Window::Invalidate(WindowState* windowState)
{
    XClearWindow(m_display, m_window);
    m_windowState = windowState;

    DrawWindowState(WindowRect(m_windowDimensions.second, 0, m_windowDimensions.first, 0),
                    windowState);
}
std::string cpplot2d::Plot2D::X11Window::GetTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", std::localtime(&t));
    return buf;
}
void cpplot2d::Plot2D::X11Window::RunEventLoop()
{
    m_running = true;

    while (m_running)
    {
        XEvent ev;
        XNextEvent(m_display, &ev);

        switch (ev.type)
        {
            case Expose:
                if (m_activeDropdown && ev.xexpose.window == m_activeDropdown->win)
                {
                    DrawDropdownMenu(m_activeDropdown);
                }
                else if (ev.xexpose.window == m_window && ev.xexpose.count == 0)
                {
                    if (m_windowState) Invalidate(m_windowState);
                }
                break;

            case ConfigureNotify:
                // Window resized
                // Resize back buffer
                m_windowDimensions = Dimension2d(ev.xconfigure.width, ev.xconfigure.height);
                if (m_backBuffer) XFreePixmap(m_display, m_backBuffer);

                m_backBuffer =
                    XCreatePixmap(m_display, m_window, ev.xconfigure.width, ev.xconfigure.height,
                                  DefaultDepth(m_display, m_screen));

                if (OnResizeCallback) OnResizeCallback();
                break;

            case ButtonPress:

                if (ev.xbutton.window == m_window)
                {
                    if (ev.xbutton.y < m_menuBarHeight)
                    {
                        HandleMenuClick(ev.xbutton.x, ev.xbutton.y);
                        break;
                    }

                    if (m_activeDropdown)
                    {
                        XUnmapWindow(m_display, m_activeDropdown->win);
                        m_activeDropdown = nullptr;
                    }

                    if (OnMouseLButtonDownCallback)
                        OnMouseLButtonDownCallback(
                            Point(ev.xbutton.x, m_windowDimensions.second - ev.xbutton.y));
                    break;
                }
                else if (m_activeDropdown && (ev.xbutton.window == m_activeDropdown->win))
                {
                    std::vector<DropdownMenuItem> items = m_activeDropdown->items;

                    // Close the dropdown first to avoid redraw issues
                    XUnmapWindow(m_display, m_activeDropdown->win);
                    m_activeDropdown = nullptr;

                    // Check if click is within any dropdown menu item
                    for (const DropdownMenuItem& item : items)
                    {
                        if (ev.xbutton.x >= item.bounds.left && ev.xbutton.x <= item.bounds.right &&
                            ev.xbutton.y >= item.bounds.top && ev.xbutton.y <= item.bounds.bottom)
                        {
                            // Invoke the callback
                            if (item.callback)
                            {
                                item.callback();
                            }

                            break;
                        }
                    }
                    break;
                }

                break;
            case ButtonRelease:
                if (OnMouseLButtonUpCallback)
                    OnMouseLButtonUpCallback(
                        Point(ev.xbutton.x, m_windowDimensions.second - ev.xbutton.y));
                break;

            case MotionNotify:
                if (ev.xmotion.window == m_window)
                {
                    XEvent nextEvent;
                    // Peek at the queue and discard all older MotionNotify events
                    while (XCheckTypedWindowEvent(m_display, m_window, MotionNotify, &nextEvent))
                    {
                        ev = nextEvent;  // Keep overwriting 'ev' with the newer data
                    }
                    if (!m_activeDropdown && ev.xmotion.y > m_menuBarHeight)
                    {
                        if (OnMouseMoveCallback)
                            OnMouseMoveCallback(
                                Point(ev.xmotion.x, m_windowDimensions.second - ev.xmotion.y));
                    }
                    break;
                }
                break;

            case KeyPress:
            case KeyRelease:
                break;

            case ClientMessage:
                if ((Atom)ev.xclient.data.l[0] == m_wmDelete)
                {
                    m_running = false;
                }
                break;
            default:
                continue;
        }
    }
}
void cpplot2d::Plot2D::X11Window::HandleMenuClick(int x, int y)
{
    for (const Menu& menu : m_menus)
    {
        WindowRect bounds = menu.bounds;
        if (x < bounds.right && x > bounds.left && y > bounds.top && y < bounds.bottom)
        {
            if (m_activeDropdown)
            {
                if (m_activeDropdown == menu.dropdown)
                {
                    // Close the dropdown if it's already open
                    XUnmapWindow(m_display, m_activeDropdown->win);
                    m_activeDropdown = nullptr;
                    return;
                }

                // Open new dropdown menu and close any existing
                XUnmapWindow(m_display, m_activeDropdown->win);
                XMoveResizeWindow(m_display, menu.dropdown->win, menu.dropdown->x, menu.dropdown->y,
                                  menu.dropdown->width, menu.dropdown->height);
                XMapWindow(m_display, menu.dropdown->win);
                XRaiseWindow(m_display, menu.dropdown->win);
                m_activeDropdown = menu.dropdown;
                return;
            }

            // Open the dropdown menu
            XMoveResizeWindow(m_display, menu.dropdown->win, menu.dropdown->x, menu.dropdown->y,
                              menu.dropdown->width, menu.dropdown->height);
            XMapWindow(m_display, menu.dropdown->win);
            XRaiseWindow(m_display, menu.dropdown->win);
            m_activeDropdown = menu.dropdown;
            return;
        }
    }

    // Clicked outside any menu, close any active dropdown
    if (m_activeDropdown)
    {
        XUnmapWindow(m_display, m_activeDropdown->win);
        m_activeDropdown = nullptr;
    }
}
void cpplot2d::Plot2D::X11Window::InvalidateRegion(const WindowRect& rect, WindowState* windowState)
{
    m_windowState = windowState;
    DrawWindowState(rect, windowState);
}
bool cpplot2d::Plot2D::X11Window::BrowseForFolder(std::string& outFolder)
{
    // Try to call Zenity
    // Use 'which' to check if it exists first to avoid shell errors
    FILE* check = popen("which zenity 2>/dev/null", "r");
    bool hasZenity = (check && fgetc(check) != EOF);
    if (check) pclose(check);

    if (hasZenity)
    {
        FILE* f = popen(
            "zenity --file-selection --save --confirm-overwrite "
            "--directory --title='Choose a Directory'",
            "r");
        if (f)
        {
            char path[1024];
            if (fgets(path, sizeof(path), f))
            {
                std::string s(path);
                if (!s.empty() && s.back() == '\n') s.pop_back();
                pclose(f);
                if (!s.empty())
                {
                    outFolder = s;
                    return true;
                }
            }
            pclose(f);
        }
        return false;
    }

    // No Zenity, fallback to default directory
    outFolder = "";
    return true;
}
cpplot2d::Plot2D::WindowRect cpplot2d::Plot2D::X11Window::GetRect()
{
    // Only return content rect, do not include top menu bar
    return GetContentRect();
}
uint8_t cpplot2d::Plot2D::X11Window::ExtractChannel(unsigned long pixel, unsigned long mask)
{
    if (mask == 0) return 0;

    int shift = 0;
    while ((mask & 1) == 0)
    {
        mask >>= 1;
        ++shift;
    }

    unsigned long value = (pixel >> shift) & mask;

    // Normalize to 8-bit
    return static_cast<uint8_t>((value * 255) / mask);
}

bool cpplot2d::Plot2D::X11Window::SaveScreenshot(const std::string& fileName)
{
    // Get directory
    std::string folder;
    if (!BrowseForFolder(folder))
    {
        return true;
    }
    // Force redraw to clear dropdown
    Invalidate(m_windowState);

    // Create image
    WindowRect rect = GetContentRect();
    Dimension2d size = rect.Size();
    XImage* img = XGetImage(m_display, m_window, 0, m_menuBarHeight, size.first, size.second,
                            AllPlanes, ZPixmap);
    if (!img)
    {
        return false;
    }

    std::vector<uint8_t> rgb;
    rgb.resize(static_cast<size_t>(static_cast<long>(size.first * size.second * 3)));

    unsigned long redMask = 0x00FF0000;
    unsigned long greenMask = 0x0000FF00;
    unsigned long blueMask = 0x000000FF;
    for (int y = 0; y < size.second; ++y)
    {
        for (int x = 0; x < size.first; ++x)
        {
            unsigned long p = XGetPixel(img, x, y);

            uint8_t r = ExtractChannel(p, redMask);
            uint8_t g = ExtractChannel(p, greenMask);
            uint8_t b = ExtractChannel(p, blueMask);

            int idx = (y * size.first + x) * 3;
            rgb[idx + 0] = r;
            rgb[idx + 1] = g;
            rgb[idx + 2] = b;
        }
    }

    std::ofstream ofs(folder + "/" + fileName, std::ios::binary);

    // P6 = Binary RGB, width, height, 255 = max color value
    ofs << "P6\n" << size.first << " " << size.second << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(rgb.data()), static_cast<long>(rgb.size()));
    ofs.close();

    XDestroyImage(img);

    return true;
}

inline unsigned long cpplot2d::Plot2D::X11Window::ToX11Pixel(const cpplot2d::Color& c)
{
    uint32_t key = (c.r << 16) | (c.g << 8) | c.b;

    auto it = m_colorCache.find(key);
    if (it != m_colorCache.end()) return it->second;

    XColor xcol;
    xcol.red = c.r * 257;
    xcol.green = c.g * 257;
    xcol.blue = c.b * 257;
    xcol.flags = DoRed | DoGreen | DoBlue;

    XAllocColor(m_display, DefaultColormap(m_display, m_screen), &xcol);

    m_colorCache[key] = xcol.pixel;
    return xcol.pixel;
}
XFontStruct* cpplot2d::Plot2D::X11Window::GetFontOfSize(int size)
{
    auto it = m_sizeToFontCache.find(size);
    if (it != m_sizeToFontCache.end()) return it->second;

    char fontSpec[256];
    snprintf(fontSpec, sizeof(fontSpec), "-*-helvetica-medium-r-normal--0-%d-100-100-p-0-iso8859-1",
             size * 10);
    XFontStruct* font = XLoadQueryFont(m_display, fontSpec);
    if (font)
    {
        m_sizeToFontCache[size] = font;
        return font;
    }

    // Try fallback
    snprintf(fontSpec, sizeof(fontSpec), "-*-*-medium-r-normal--0-%d-100-100-p-0-iso8859-1",
             size * 10);
    font = XLoadQueryFont(m_display, fontSpec);
    if (font)
    {
        m_sizeToFontCache[size] = font;
        return font;
    }

    throw std::runtime_error("Failed to load font of size " + std::to_string(size));
}
void cpplot2d::Plot2D::X11Window::DrawWindowState(const WindowRect& rect, WindowState* windowState)
{
    Dimension2d size = m_windowDimensions;

    // Fill background
    XSetForeground(m_display, m_gc, ToX11Pixel(windowState->background));

    XFillRectangle(m_display, m_backBuffer, m_gc, rect.left, size.second - rect.top,
                   rect.right - rect.left, rect.top - rect.bottom);

    // Draw polylines
    std::vector<XPoint> pts;
    for (const GuiPolyline& polyline : windowState->polylines)
    {
        pts.reserve(polyline.points.size());

        for (auto& p : polyline.points)
        {
            pts.push_back(
                {static_cast<short>(p.first), static_cast<short>(size.second - p.second)});
        }

        XSetForeground(m_display, m_gc, ToX11Pixel(polyline.color));
        XDrawLines(m_display, m_backBuffer, m_gc, pts.data(), static_cast<int>(pts.size()),
                   CoordModeOrigin);
        pts.clear();
    }

    // Draw Lines
    for (const GuiLine& line : windowState->lines)
    {
        XSetForeground(m_display, m_gc, ToX11Pixel(line.color));

        XDrawLine(m_display, m_backBuffer, m_gc, line.p1.first, size.second - line.p1.second,
                  line.p2.first, size.second - line.p2.second);
    }

    // Draw rects
    for (const GuiRect& guiRect : windowState->rects)
    {
        XSetForeground(m_display, m_gc, ToX11Pixel(guiRect.borderColor));

        XDrawRectangle(m_display, m_backBuffer, m_gc, guiRect.topLeft.first,
                       size.second - guiRect.topLeft.second,
                       guiRect.bottomRight.first - guiRect.topLeft.first,
                       guiRect.topLeft.second - guiRect.bottomRight.second);
    }

    // Draw Circles
    for (const GuiCircle& circle : windowState->circles)
    {
        XSetForeground(m_display, m_gc, ToX11Pixel(circle.fillColor));

        int x = circle.center.first - circle.radius;
        int y = size.second - (circle.center.second + circle.radius);
        int d = circle.radius * 2;

        XFillArc(m_display, m_backBuffer, m_gc, x, y, d, d, 0, 360 * 64);
    }

    // Draw text
    for (const GuiText& text : windowState->text)
    {
        XFontStruct* font = GetFontOfSize(text.size);
        XSetForeground(m_display, m_gc, ToX11Pixel(text.color));
        XSetFont(m_display, m_gc, font->fid);
        int width = XTextWidth(font, text.text.c_str(), static_cast<int>(text.text.length()));
        int posX = text.pos.first;

        if (text.alignment == Alignment::RIGHT)
        {
            posX -= width;
        }

        if (text.orientation == Orientation::VERTICAL)
        {
            DrawTextVertical(font, ToX11Pixel(windowState->background), ToX11Pixel(text.color),
                             posX, size.second - text.pos.second, text.text.c_str());
        }
        else
        {
            XDrawString(m_display, m_backBuffer, m_gc, posX, size.second - text.pos.second,
                        text.text.c_str(), static_cast<int>(text.text.length()));
        }
    }

    // Draw at the end so it appears on top
    DrawMenuBar();

    // Draw back buffer
    XCopyArea(m_display, m_backBuffer, m_window, m_gc, 0, 0, size.first, size.second, 0, 0);
    XSync(m_display, False);
    XFlush(m_display);
}
void cpplot2d::Plot2D::X11Window::DrawTextVertical(XFontStruct* font, unsigned long background,
                                                   unsigned long color, int x, int y,
                                                   const char* str)
{
    int len = static_cast<int>(strlen(str));
    if (len == 0) return;

    int width = XTextWidth(font, str, len);
    int height = font->ascent + font->descent;
    int depth = DefaultDepth(m_display, 0);

    // Create a temporary Pixmap
    Pixmap tmp = XCreatePixmap(m_display, m_window, width, height, depth);
    XSetForeground(m_display, m_gc, background);
    XFillRectangle(m_display, tmp, m_gc, 0, 0, width, height);

    XSetForeground(m_display, m_gc, color);
    XSetFont(m_display, m_gc, font->fid);
    XDrawString(m_display, tmp, m_gc, 0, font->ascent, str, len);

    XImage* horizImg = XGetImage(m_display, tmp, 0, 0, width, height, AllPlanes, ZPixmap);

    // Allocate memory for the vertical image
    // Swapping width/height. Assume 4 bytes per pixel (32-bit) for simplicity here
    char* vertData = (char*)malloc(static_cast<size_t>(static_cast<long>(height * width * 4)));
    XImage* vertImg = XCreateImage(m_display, DefaultVisual(m_display, 0), depth, ZPixmap, 0,
                                   vertData, height, width, 32, 0);

    for (int ix = 0; ix < width; ix++)
    {
        for (int iy = 0; iy < height; iy++)
        {
            unsigned long pixel = XGetPixel(horizImg, ix, iy);

            // Counter-clockwise 90: (ix, iy) -> (iy, (width-1) - ix)
            XPutPixel(vertImg, iy, (width - 1) - ix, pixel);
        }
    }

    // Push to backbuffer
    XPutImage(m_display, m_backBuffer, m_gc, vertImg, 0, 0, x, y - width, height, width);

    XDestroyImage(horizImg);
    XDestroyImage(vertImg);
    XFreePixmap(m_display, tmp);
}
inline cpplot2d::Plot2D::WindowRect cpplot2d::Plot2D::X11Window::GetContentRect()
{
    return WindowRect(m_windowDimensions.second - m_menuBarHeight, 0, m_windowDimensions.first, 0);
}
void cpplot2d::Plot2D::X11Window::DrawMenuBar()
{
    int x = 4;
    int y = 0;
    unsigned long menuBgColor = ToX11Pixel(m_menuColor);
    unsigned long textColor = ToX11Pixel(Color(0, 0, 0));

    // Background color
    XSetForeground(m_display, m_gc, menuBgColor);
    XFillRectangle(m_display, m_backBuffer, m_gc, 0, 0, m_windowDimensions.first, m_menuBarHeight);

    for (auto& menu : m_menus)
    {
        int textWidth =
            XTextWidth(m_menuFont, menu.title.c_str(), static_cast<int>(menu.title.length()));

        menu.bounds = {0, x, x + textWidth + 16, m_menuBarHeight};

        // Text color
        XSetForeground(m_display, m_gc, textColor);
        XDrawString(m_display, m_backBuffer, m_gc, x + 8, 16, menu.title.c_str(),
                    static_cast<int>(menu.title.length()));

        x += textWidth + 24;
    }
}

bool cpplot2d::Plot2D::X11Window::LoadFont()
{
    for (const std::string& name : m_fontFallbacks)
    {
        m_menuFont = XLoadQueryFont(m_display, name.c_str());
        if (m_menuFont) return true;
    }
    return false;
}

#endif  // __linux__
#endif  // CPPLOT2D_IMPLEMENTATION
}  // namespace cpplot2d