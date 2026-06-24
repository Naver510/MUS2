#pragma once

struct LowPassFilter_t {
    float alpha = 0.0f;
    float output = 0.0f;
};

void LowPassFilter_Init(LowPassFilter_t* LPF, float alpha);
float LowPassFilter_Update(LowPassFilter_t* LPF, float input);
