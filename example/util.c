#include "util.h"

double sin_wave(WaveParam_t *wp, double t)
{
  return wp->A * sin(2 * M_PI / wp->T * t) + wp->b;
}

double cos_wave(WaveParam_t *wp, double t)
{
  return wp->A * -cos(2 * M_PI / wp->T * t) + wp->b + wp->A;
}

double square_wave(WaveParam_t *wp, double t)
{
  double v = 0;
  if (t < (wp->T / 2.0))
  {
    v = wp->A + wp->b;
  }
  else
  {
    v = -wp->A + wp->b;
  }
  return v;
}

double triangular_wave(WaveParam_t *wp, double t)
{
  double v = 0;
  if (t < (wp->T / 2.0))
  {
    v = (wp->A / (wp->T / 2.0)) * t + wp->b;
  }
  else
  {
    v = (wp->A / (wp->T / 2.0)) * (wp->T - t) + wp->b;
  }
  return v;
}

double P_controller(PIDParam_t *pid, double err)
{
  for (uint16_t i = 0; i < 2; i++)
  {
    pid->err[i] = pid->err[i + 1];
  }
  pid->err[2] = err;
  return pid->kp * pid->err[2];
}

double PI_controller(PIDParam_t *pid, double err)
{
  for (uint16_t i = 0; i < 2; i++)
  {
    pid->err[i] = pid->err[i + 1];
  }
  pid->err[2] = err;
  pid->intergral += pid->err[2];
  pid->intergral = LIMITING(pid->intergral, pid->intergral_lim[0], pid->intergral_lim[1]);
  return pid->kp * pid->err[2] + pid->ki * pid->intergral;
}