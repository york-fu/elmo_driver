#ifndef _util_h_
#define _util_h_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

double get_sin_wave(double A, double T, double b, double dt);
double get_cos_wave(double A, double T, double b, double dt);
double get_square_wave(double A, double T, double b, double dt);
double get_triangular_wave(double A, double T, double b, double dt);
void average_filter(double *data, uint32_t size, double *result);
void median_filter(double *data, uint32_t size, double *result);
void median_average_filter(double *data, uint32_t size, double *result);

#endif