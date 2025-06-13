#ifndef COLOR_PALETTE_HPP
#define COLOR_PALETTE_HPP

#include <string>
#include <vector>
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

struct Color
{
    int r, g, b;
};

static const std::vector<std::string> class_name = {
    "Pedestrian", "Rubbercorn", "Barrel", "Robot", "Car", "Suv_Forklift", "Truck_Van", "Exconst", "Excavator", "Pump_Crane", "Fence"
};

static const std::vector<Color> palette = {
    {255, 255, 0},    // Pedestrian - Bright Yellow
    {255, 0, 228},    // Rubbercorn - Magenta
    {0, 255, 0},      // Barrel - Lime Green
    {255, 165, 0},    // Robot - Bright Orange
    {0, 128, 255},    // Car - Sky Blue
    {255, 0, 0},      // Suv_Forklift - Red
    {0, 255, 255},    // Truck_Van - Cyan
    {128, 0, 255},    // Exconst - Purple
    {255, 105, 180},  // Excavator - Hot Pink
    {0, 255, 128},    // Pump_Crane - Aquamarine
    {255, 215, 0}     // Fence - Golden Yellow
};

inline std_msgs::msg::ColorRGBA nomalized_color(const Color& c, float a = 1.0f) {
    std_msgs::msg::ColorRGBA color;
    color.r = c.r / 255.0f;
    color.g = c.g / 255.0f;
    color.b = c.b / 255.0f;
    color.a = a;
    return color;
}

#endif
