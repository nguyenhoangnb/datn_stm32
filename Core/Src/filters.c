#include "filters.h"
#include <string.h>

void MovingAverage_Init(MovingAverageFilter* filter) {
    for(uint8_t i = 0; i < FILTER_SIZE; i++) {
        filter->values[i] = 0.0f;
    }
    filter->sum = 0.0f;
    filter->count = 0;
    filter->index = 0;
}

float MovingAverage_Update(MovingAverageFilter* filter, float new_value) {
    if(filter->count == FILTER_SIZE) {
        filter->sum -= filter->values[filter->index];
    } else {
        filter->count++;
    }
    
    filter->values[filter->index] = new_value;
    filter->sum += new_value;
    filter->index = (filter->index + 1) % FILTER_SIZE;
    
    return filter->sum / filter->count;
}

void MovingAverage_Reset(MovingAverageFilter* filter) {
    MovingAverage_Init(filter);
}

void MedianFilter_Init(MedianFilter* filter) {
    for(uint8_t i = 0; i < FILTER_SIZE; i++) {
        filter->values[i] = 0.0f;
    }
    filter->count = 0;
    filter->index = 0;
}

float MedianFilter_Update(MedianFilter* filter, float new_value) {
    // Add new value
    filter->values[filter->index] = new_value;
    if(filter->count < FILTER_SIZE) {
        filter->count++;
    }
    filter->index = (filter->index + 1) % FILTER_SIZE;
    
    // Sort values
    float temp[FILTER_SIZE];
    for(uint8_t i = 0; i < filter->count; i++) {
        temp[i] = filter->values[i];
    }
    
    // Simple bubble sort
    for(uint8_t i = 0; i < filter->count-1; i++) {
        for(uint8_t j = 0; j < filter->count-i-1; j++) {
            if(temp[j] > temp[j+1]) {
                float t = temp[j];
                temp[j] = temp[j+1];
                temp[j+1] = t;
            }
        }
    }
    
    return temp[filter->count/2];
}

void MedianFilter_Reset(MedianFilter* filter) {
    MedianFilter_Init(filter);
}
