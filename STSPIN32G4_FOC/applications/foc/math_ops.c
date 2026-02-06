
#include "math_ops.h"
#include"LUT.h"

float fmaxf(float x, float y) {
    /// Returns maximum of x, y ///
    return (((x) > (y)) ? (x) : (y));
}

float fminf(float x, float y) {
    /// Returns minimum of x, y ///
    return (((x) < (y)) ? (x) : (y));
}

float fmaxf3(float x, float y, float z) {
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z) {
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

float roundf(float x) {
    /// Returns nearest integer ///

    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
}

void limit_norm(float *x, float *y, float limit) {
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if (norm > limit) {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

void limit(float *x, float min, float max) {
    *x = fmaxf(fminf(*x, max), min);
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}


float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

const float Multiplier = 81.4873308631f;

float FastSin(float theta) {
    while (theta < 0.0f) theta += 6.28318530718f;
    while (theta >= 6.28318530718f) theta -= 6.28318530718f;
    return SinTable[(int) (Multiplier * theta)];
}

float FastCos(float theta) {
    return FastSin(1.57079632679f - theta);
}
