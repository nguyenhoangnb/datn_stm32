#ifndef FILTERS_H
#define FILTERS_H

#include "stm32f4xx.h"

#define FILTER_SIZE 5

typedef struct {
    float values[FILTER_SIZE];
    float sum;
    uint8_t count;
    uint8_t index;
} MovingAverageFilter;

typedef struct {
    float values[FILTER_SIZE];
    uint8_t count;
    uint8_t index;
} MedianFilter;

void MovingAverage_Init(MovingAverageFilter* filter);
float MovingAverage_Update(MovingAverageFilter* filter, float new_value);
void MedianFilter_Init(MedianFilter* filter);
float MedianFilter_Update(MedianFilter* filter, float new_value);

#endif
