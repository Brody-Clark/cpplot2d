# cxpplot

> [!IMPORTANT]
> **Project Status: In Progress**
> _This project is actively being developed. Some features are incomplete or experimental._

## About

This is a cross-platform, header-only 2-D plotting library for C++ meant to provide basic line plotting for quick visualization of real-number datasets. It is intended to be simple to use and is designed to work without needing any third party libraries or tools installed. It also includes a save feature to save plots as .png files.

## Requirements

- **Operating System**: Windows, MacOs, Linux
- **Compiler**: Any C++ compiler

## Usage

1. **Include the cxpplot header found in `src/`**

2. **For Windows: Use the `WinMain` function as the entry point for your application and Build your project with the Windows subsystem:**
    - In your project settings, set the **Linker System** to **Windows**.
3. **For Apple idk yet**
4. **For Linux idk yet**

## Examples

### Windows

```cpp

#include "cxpplot.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int CmdShow)
{
    // Create dataset as 2-D vector of any numeric type
    std::vector<float> x;
    std::vector<float> y;

    // Call plot function with default labels ('x', 'y') and default tile ('Plot')
    cxpplot::Plot2D plot(x, y);
    plot.Show();

    // Call plot function with custom axis labels and title
    cxpplot::Plot2D plot(x, y, "distance", "time", "Distance vs Time");
    plot.Show();
}

```

### Apple

```cpp

#include "cxpplot.h"

int main(int argc, const char * argv[])
{
    // Create dataset as 2-D vector of any numeric type
    std::vector<float> x;
    std::vector<float> y;

    // Call plot function with default labels ('x', 'y') and default tile ('Plot')
    cxpplot::Plot2D plot(x, y);
    plot.Show();

    // Call plot function with custom axis labels and title
    cxpplot::Plot2D plot(x, y, "distance", "time", "Distance vs Time");
    plot.Show();
}

```

### Linux

```cpp
TODO
```

**Sample plot:**

!["Demo Screenshot"](PlotDemo.png)

## License

MIT License with Attribution Clause  

Copyright (c) 2025 Brody Clark  

Permission is hereby granted, free of charge, to any person obtaining a copy  
of this software and associated documentation files (the "Software"), to deal  
in the Software without restriction, including without limitation the rights  
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell  
copies of the Software, subject to the following conditions:  

1. **Attribution Requirement**  
   Any use of this Software, including in original or modified form, must include  
   proper attribution to the original author(s) as follows:  
   - The author's name must be credited in the accompanying documentation, README,  
     or other relevant written materials.  
   - If the Software is used in a user-facing application (such as a website, game,  
     or graphical software), an attribution notice must be visible within the UI,  
     such as in an "About" section or equivalent location.  

2. **Derivative Works**  
   Any modified or derivative works of this Software must include a prominent notice  
   stating that the work is derived from this Software and must also comply with  
   the attribution requirements above.  

3. **Preservation of License**  
   This license notice, including the attribution requirement, must be included in  
   all copies or substantial portions of the Software.  

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,  
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A  
PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT  
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION  
OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE  
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  
