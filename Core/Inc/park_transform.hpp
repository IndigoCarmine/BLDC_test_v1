#pragma once

#include "data.h"
#include "math.h"

// theta is delta angle
// vector is also used as output
void park_transform(Vector2D *vector, float theta)
{
    Vector2D vector_prev = *vector;
    vector->x = vector_prev.x * cos(theta) + vector_prev.y * sin(theta);
    vector->y = -vector_prev.x * sin(theta) + vector_prev.y * cos(theta);
}

// theta is delta angle
// vector is also used as output
void rev_park_transform(Vector2D *vector, float theta)
{
    Vector2D vector_prev = *vector;
    vector->x = vector_prev.x * cos(theta) - vector_prev.y * sin(theta);
    vector->y = vector_prev.x * sin(theta) + vector_prev.y * cos(theta);
}
