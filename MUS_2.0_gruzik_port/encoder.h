#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// 4× quadrature encoder — software ISR implementation (CHANGE on A and B).
//
// ENC1 (left)  : A = GPIO6, B = GPIO7
// ENC2 (right) : A = GPIO8, B = GPIO9
//
// Counts are signed 32-bit and accumulate indefinitely.
// Call Encoders_Reset() to zero both counters.
// ---------------------------------------------------------------------------

// Call once in setup() before starting FreeRTOS tasks
void Encoders_Init(void);

// Returns signed 32-bit count
int32_t Encoder1_GetCount(void);
int32_t Encoder2_GetCount(void);

// ISR fire counters — incremented unconditionally each ISR call.
// If these stay 0 while wheels spin: GPIO pins receive no CHANGE events
// (encoder not powered, wrong pins, or pins stuck).
// If these increment but Encoder*_GetCount stays 0: direction cancellation.
extern volatile uint32_t isr1_count;
extern volatile uint32_t isr2_count;

// Zero both counters atomically
void Encoders_Reset(void);

#endif // ENCODER_H
