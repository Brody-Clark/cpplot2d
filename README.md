# pltcpp

This is a lightweight, header-only 2-D C++ plotting library meant to provide minimalistic line plotting for quick visualization of data. It is designed to work without needing any third party libraries or tools installed. It also includes a print and export feature to print plots to pdf and export to CSV files respectively.

## How to use

To use in your project, simply add the header file found in the `src` folder to your own project folder and add `#include "pltcpp.hpp"` to the source files that require it. The following code snippet demonstrates how to plot:

```cpp

#include "pltcpp.hpp"

// Create dataset using pltPoint structures
std::vector<pltcpp::pltPoint> points;

// Call plot function with default labels ('x', 'y')
pltcpp::plt::Plot(points);

// Call plot function with custom labels
pltcpp::plt::Plot(points, "time", "distance");

```
