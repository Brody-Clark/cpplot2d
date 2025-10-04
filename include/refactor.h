#pragma once

#include <vector>
#include <utility>
#include <functional>
#include <string>
#include <string_view>

using Point = std::pair<int, int>();

enum class Orientation : uint8_t
{
    HORIZONTAL = 0,
    VERTICAL = 1
};
class Window
{
   public:
    virtual void DrawPoint(int x, int y, Color color) = 0;
    virtual void DrawPoints(std::vector<std::pair<int, int>>, Color color) = 0;
    virtual void DrawLine(std::pair<int, int> start, std::pair<int, int> end, Color color) = 0;
    // virtual void DrawLines(std::vector<std::pair<int, int>>) = 0;
    virtual void OnResize(std::function<void> callback) = 0;
    virtual void OnMouseHover(std::function<void(Point)> callback) = 0;
    virtual void DrawText(std::string_view text, int spacing, int size = 1,
                          Orientation orientation = 0) = 0;

    virtual void Draw() = 0;
};
class GraphicsContext
{
   public:
    virtual bool Init() = 0;
    virtual bool Shutdown() = 0;
    virtual Window CreateWindow() = 0;  // TODO: add params for initial size and location
};

class Win32GraphicsContext : public GraphicsContext
{
};
class Win32Window : public Window
{
};