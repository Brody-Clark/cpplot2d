# !["Logo"](C2PPlot.png)

This is a lightweight, easy to use, header-only 2-D C++ plotting library meant to provide basic line plotting for quick visualization of data. It is designed to work without needing any third party libraries or tools installed. It also includes a print and export feature to save plots as .png files and export to CSV files respectively.

## How to use

To use in your project, simply add the header file found in the `c2pplot/src` folder to your own project folder and add `#include "c2pplot.h"` to the source files that require it. The following code snippet demonstrates how to call the plot method:

```cpp

#include "c2pplot.h"

// Create dataset as 2-D vector of any numeric type
std::vector<std::vector<float>> points;

// Call plot function with default labels ('x', 'y') and default tile ('Plot')
c2pplot::plt::Plot(points);

// Call plot function with custom axis labels and title
c2pplot::plt::Plot(points, "time", "distance", "Distance vs Time");

```
