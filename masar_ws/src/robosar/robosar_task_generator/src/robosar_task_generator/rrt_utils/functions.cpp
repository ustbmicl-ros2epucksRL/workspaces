#include "functions.h"

// rdm class, for gentaring random flot numbers
rdm::rdm() { i = time(0); }

float rdm::randomize()
{
    i = i + 1;
    srand(i);
    return float(rand()) / float(RAND_MAX);
}

// Norm function
float Norm(float x1, float y1, float x2, float y2)
{
    return pow((pow((x2 - x1), 2) + pow((y2 - y1), 2)), 0.5);
}

std::pair<float, float> Steer(std::pair<float, float> &x_nearest, std::pair<float, float> &x_rand, float eta)
{
    std::pair<float, float> x_new;

    if (Norm(x_nearest.first, x_nearest.second, x_rand.first, x_rand.second) <= eta)
    {
        x_new = x_rand;
    }
    else
    {
        float m = (x_rand.second - x_nearest.second) / (x_rand.first - x_nearest.first);
        if (x_rand.first == x_nearest.first)
        {
            x_new = {x_nearest.first, x_nearest.second + eta};
        }
        x_new.first = (sign(x_rand.first - x_nearest.first)) * (sqrt((pow(eta, 2)) / ((pow(m, 2)) + 1))) + x_nearest.first;
        x_new.second = m * (x_new.first - x_nearest.first) + x_nearest.second;
    }
    return x_new;
}

// sign function
float sign(float n)
{
    if (n < 0.0)
    {
        return -1.0;
    }
    else
    {
        return 1.0;
    }
}
