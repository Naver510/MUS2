#include "encoder.h"
#include "robot_config.h"
#include <driver/gpio.h>

// ---------------------------------------------------------------------------
// 2× quadrature — A-only ANYEDGE ISR registered via ESP-IDF gpio API.
//
// WHY ESP-IDF API instead of attachInterrupt():
//   attachInterrupt() internally calls gpio_install_isr_service(0) — without
//   the IRAM flag.  When the WiFi stack causes flash cache misses (serving
//   pages, TLS crypto, etc.), the GPIO ISR dispatch table (in flash) cannot
//   run.  GPIO interrupts are silently deferred/dropped the entire time flash
//   is busy → encoders appear completely dead while WiFi is active.
//
//   By calling gpio_install_isr_service(ESP_INTR_FLAG_IRAM) BEFORE WiFi
//   starts (we are called from setup() before WebServer_Init()), the dispatch
//   table is placed in IRAM and ISRs fire even during flash cache misses.
//
// WHY 2× (A only, no B ISR):
//   At high RPM, FreeRTOS briefly masks GPIO interrupts during task switches.
//   If A changes AND B changes before either ISR runs, the A-ISR reads B's
//   new state → wrong direction → A-ISR and B-ISR cancel → net count ≈ 0.
//   A-only ISR has no B-ISR to cancel it, so it always accumulates.
//
// WHY NO portMUX in ISR:
//   32-bit aligned volatile int32_t stores are atomic on Xtensa LX7.
//   portENTER_CRITICAL_ISR would mask interrupts across both cores, slowing
//   the ISR unnecessarily.  Encoder1_GetCount() does a simple volatile read.
//
// CPR: 7 PPR × 2 A-edges (rise+fall) × 10 gear ratio = 140.
// Wiring: ENC1 A=GPIO6, B=GPIO7   ENC2 A=GPIO8, B=GPIO9
// ---------------------------------------------------------------------------

static volatile int32_t enc1_raw = 0;
static volatile int32_t enc2_raw = 0;

// ISR call counters — increment unconditionally so we know if the ISR fires at all
volatile uint32_t isr1_count = 0;
volatile uint32_t isr2_count = 0;

// ISR signature required by Arduino attachInterrupt: void (*)(void)
static void IRAM_ATTR isr_enc1(void) {
    isr1_count++;   // always increment so we know ISR is firing even if count stays 0
    int a = gpio_get_level((gpio_num_t)PIN_ENC1_A);
    int b = gpio_get_level((gpio_num_t)PIN_ENC1_B);
    if (a != b) enc1_raw++;
    else        enc1_raw--;
}

static void IRAM_ATTR isr_enc2(void) {
    isr2_count++;
    int a = gpio_get_level((gpio_num_t)PIN_ENC2_A);
    int b = gpio_get_level((gpio_num_t)PIN_ENC2_B);
    if (a != b) enc2_raw++;
    else        enc2_raw--;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void Encoders_Init(void)
{
    // ── GPIO config ─────────────────────────────────────────────────────────
    // We use Arduino's pinMode because it correctly handles the ESP32-S3 IOMUX
    // (routing the physical pad to the GPIO matrix), whereas raw gpio_config
    // sometimes misses IOMUX setup if the pin was in a special state.
    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);
    pinMode(PIN_ENC2_A, INPUT_PULLUP);
    pinMode(PIN_ENC2_B, INPUT_PULLUP);

    // ── ISR service with IRAM flag ───────────────────────────────────────────
    // Arduino core sometimes installs the ISR service automatically without the
    // IRAM flag before setup() even runs. We must uninstall it first, then
    // reinstall it with the IRAM flag so our encoders survive WiFi flash misses.
    gpio_uninstall_isr_service();
    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        Serial.printf("[ENC] WARN: gpio_install_isr_service failed: %d\n", (int)ret);
    }

    // Attach the interrupts (CHANGE = ANYEDGE) using Arduino API.
    // Because we just installed the ISR service with IRAM flag above,
    // attachInterrupt will piggyback on it and keep our IRAM safety!
    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), isr_enc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), isr_enc2, CHANGE);

    Serial.printf("[ENC] ESP-IDF+IRAM ISR OK — ENC1 A=GPIO%d B=GPIO%d, ENC2 A=GPIO%d B=GPIO%d\n",
                  PIN_ENC1_A, PIN_ENC1_B, PIN_ENC2_A, PIN_ENC2_B);
    Serial.printf("[ENC] CPR=%d  (7PPR x 2 edges x 10 gear)\n",
                  (int)ROBOT_ENCODER_COUNTS_PER_WHEEL_REV);
}

int32_t Encoder1_GetCount(void)
{
    return enc1_raw;   // volatile: always re-read from memory, never cached in register
}

int32_t Encoder2_GetCount(void)
{
    return enc2_raw;
}

void Encoders_Reset(void)
{
    enc1_raw = 0;
    enc2_raw = 0;
}
