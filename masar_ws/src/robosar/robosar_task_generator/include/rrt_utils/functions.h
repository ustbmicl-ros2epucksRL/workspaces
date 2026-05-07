#ifndef functions_H
#define functions_H

#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

// rdm class, for gentaring random flot numbers
class rdm
{
    int i;

public:
    rdm();
    float randomize();
};

// Norm function prototype
float Norm(float x1, float y1, float x2, float y2);

std::pair<float, float> Steer(std::pair<float, float> &x_nearest, std::pair<float, float> &x_rand, float eta);

// sign function prototype
float sign(float);

#endif