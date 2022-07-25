#include "util.h"

double get_sin_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  out = A * sin(2 * M_PI / T * time) + b;
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double get_cos_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  out = A * -cos(2 * M_PI / T * time) + b + A;
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double get_square_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = A + b;
  }
  else
  {
    out = -A + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

double get_triangular_wave(double A, double T, double b, double dt)
{
  static double time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = (A / (T / 2.0)) * time + b;
  }
  else
  {
    out = (A / (T / 2.0)) * (T - time) + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

void average_filter(double *data, uint32_t size, double *result)
{
  double sum = 0;
  for (uint32_t i = 0; i < size - 1; i++)
  {
    data[i] = data[i + 1];
    sum += data[i + 1];
  }
  data[size - 1] = *result;
  sum += *result;
  *result = (sum / size);
}

void median_filter(double *data, uint32_t size, double *result)
{
  double sort_data[size];
  for (uint32_t i = 0; i < size - 1; i++)
  {
    data[i] = data[i + 1];
    sort_data[i] = data[i];
  }
  data[size - 1] = *result;
  sort_data[size - 1] = data[size - 1];

  uint32_t i, j;
  double temp;
  for (j = 0; j < size - 1; j++)
  {
    for (i = 0; i < size - j - 1; i++)
    {
      if (sort_data[i] < sort_data[i + 1])
      {
        temp = sort_data[i];
        sort_data[i] = sort_data[i + 1];
        sort_data[i + 1] = temp;
      }
    }
  }

  if ((size & 1) > 0)
  {
    *result = sort_data[(size - 1) / 2];
  }
  else
  {
    *result = (sort_data[size / 2 - 1] + sort_data[size / 2]) / 2.0;
  }
}

void median_average_filter(double *data, uint32_t size, double *result)
{
  if (size > 4)
  {
    double sort_data[size];
    for (uint32_t i = 0; i < size - 1; i++)
    {
      data[i] = data[i + 1];
      sort_data[i] = data[i];
    }
    data[size - 1] = *result;
    sort_data[size - 1] = data[size - 1];

    uint32_t i, j;
    double temp;
    for (j = 0; j < size - 1; j++)
    {
      for (i = 0; i < size - j - 1; i++)
      {
        if (sort_data[i] < sort_data[i + 1])
        {
          temp = sort_data[i];
          sort_data[i] = sort_data[i + 1];
          sort_data[i + 1] = temp;
        }
      }
    }

    double sum = 0;
    for (uint32_t i = 2; i < size - 2; i++)
    {
      sum += sort_data[i];
    }

    *result = sum / (size - 4);
  }
}
