/**
 * @file util.h
 * @author york (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _util_h_
#define _util_h_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

typedef struct
{
  double A;
  double T;
  double b;
} WaveParam_t;

typedef struct
{
  double kp;
  double ki;
  double kd;
  double intergral;
  double err[3];
  double intergral_lim[2];
} PIDParam_t;

double sin_wave(WaveParam_t *wp, double t);
double cos_wave(WaveParam_t *wp, double t);
double square_wave(WaveParam_t *wp, double t);
double triangular_wave(WaveParam_t *wp, double t);

double P_controller(PIDParam_t *pid, double err);
double PI_controller(PIDParam_t *pid, double err);

#endif