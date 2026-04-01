/*
MIT License

Copyright (c) 2026 Mason Z.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->
*/

#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "icache.h"
#include "tim.h"
#include <cstring>
#include <cmath>
#include <cstdint>

extern "C" {
    void SystemClock_Config(void);
    void Error_Handler(void);
}

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

// Simulation Parameters 
#define NUM_PARTICLES           50      // particle count - adjustable
#define GRAVITY_STRENGTH        40.0f   // gravity - adjustable
#define SIM_RADIUS              8.0f    // simulation radius - do not touch
#define DT                      0.025f  // delta time - do not touch
#define PARTICLE_REPULSE        1.3f    // min distance between each particle - do not touch
#define DAMPING                 0.78f   // velocity scale on wall bounce - adjustable
#define RESTITUTION             1.35f   // Bounce reflection multiplier - adjustable
#define MUX_DWELL_US            30      // LED refresh rate - do not touch
#define PHYSICS_STEPS_PER_PASS  1       // physics steps per frame - do not touch
#define ACCEL_READ_INTERVAL     8       // sensor read every N physics steps - do not touch

// Activity / Sleep
#define ACTIVITY_THRESHOLD      50.0f    // gravity unit change to count as movement
#define SLEEP_AFTER_SECONDS     50      // seconds of inactivity before sleep

#define READS_PER_SECOND        (1000 / (ACCEL_READ_INTERVAL * 16))
#define INACTIVITY_COUNT_MAX    (SLEEP_AFTER_SECONDS * READS_PER_SECOND)

static bool is_sleeping = false;

// LED Mapping
const int8_t anode_map[256] = {
    -1, -1, -1, -1, -1,  7,  5,  9,  7, 11,  9, -1, -1, -1, -1, -1,
    -1, -1, -1,  1,  7,  3,  9,  5, 11,  7, 13,  9, 15, -1, -1, -1,
    -1, -1,  2,  7,  1,  9,  3, 11,  5, 13,  7, 15,  9, 17, -1, -1,
    -1,  4,  7,  2,  9,  1, 11,  3, 13,  5, 15,  7, 17,  9, 16, -1,
    -1,  7,  4,  9,  2, 11,  1, 13,  3, 15,  5, 17,  7, 16,  9, -1,
     7,  6,  9,  4, 11,  2, 13,  1, 15,  3, 17,  5, 16,  7, 14,  9,
     8,  9,  6, 11,  4, 13,  2, 15,  1, 17,  3, 16,  5, 14,  7,  9,
     9,  8, 11,  6, 13,  4, 15,  2, 17,  1, 16,  3, 14,  5, 12,  7,
    10, 11,  8, 13,  6, 15,  4, 17,  2, 16,  1, 14,  3, 12,  5,  7,
    11, 10, 13,  8, 15,  6, 17,  4, 16,  2, 14,  1, 12,  3, 10,  5,
    12, 13, 10, 15,  8, 17,  6, 16,  4, 14,  2, 12,  1, 10,  3,  5,
    -1, 12, 15, 10, 17,  8, 16,  6, 14,  4, 12,  2, 10,  1,  8, -1,
    -1, 15, 12, 17, 10, 16,  8, 14,  6, 12,  4, 10,  2,  8,  1, -1,
    -1, -1, 17, 12, 16, 10, 14,  8, 12,  6, 10,  4,  8,  2, -1, -1,
    -1, -1, -1, 16, 12, 14, 10, 12,  8, 10,  6,  8,  4, -1, -1, -1,
    -1, -1, -1, -1, -1, 12, 12, 10, 10,  8,  8, -1, -1, -1, -1, -1
};

const int8_t cathode_map[256] = {
    -1, -1, -1, -1, -1,  5,  7,  7,  9,  9, 11, -1, -1, -1, -1, -1,
    -1, -1, -1,  5,  3,  7,  5,  9,  7, 11,  9, 13, 11, -1, -1, -1,
    -1, -1,  5,  1,  7,  3,  9,  5, 11,  7, 13,  9, 15, 11, -1, -1,
    -1,  5,  2,  7,  1,  9,  3, 11,  5, 13,  7, 15,  9, 17, 11, -1,
    -1,  4,  7,  2,  9,  1, 11,  3, 13,  5, 15,  7, 17,  9, 16, -1,
     6,  7,  4,  9,  2, 11,  1, 13,  3, 15,  5, 17,  7, 16,  9, 14,
     7,  6,  9,  4, 11,  2, 13,  1, 15,  3, 17,  5, 16,  7, 14, 12,
     8,  9,  6, 11,  4, 13,  2, 15,  1, 17,  3, 16,  5, 14,  7, 12,
     9,  8, 11,  6, 13,  4, 15,  2, 17,  1, 16,  3, 14,  5, 12, 10,
    10, 11,  8, 13,  6, 15,  4, 17,  2, 16,  1, 14,  3, 12,  5, 10,
    11, 10, 13,  8, 15,  6, 17,  4, 16,  2, 14,  1, 12,  3, 10,  8,
    -1, 13, 10, 15,  8, 17,  6, 16,  4, 14,  2, 12,  1, 10,  3, -1,
    -1, 12, 15, 10, 17,  8, 16,  6, 14,  4, 12,  2, 10,  1,  8, -1,
    -1, -1, 12, 17, 10, 16,  8, 14,  6, 12,  4, 10,  2,  8, -1, -1,
    -1, -1, -1, 12, 16, 10, 14,  8, 12,  6, 10,  4,  8, -1, -1, -1,
    -1, -1, -1, -1, -1, 14, 10, 12,  8, 10,  6, -1, -1, -1, -1, -1
};

// Pin Mapping
struct PinMapping { GPIO_TypeDef* port; uint32_t pin_pos; };
static PinMapping pin_map[18];

static void InitPinMap() {
    pin_map[1]  = {GPIOA, 9};  pin_map[2]  = {GPIOA, 8};
    pin_map[3]  = {GPIOB, 15}; pin_map[4]  = {GPIOB, 14};
    pin_map[5]  = {GPIOB, 13}; pin_map[6]  = {GPIOB, 12};
    pin_map[7]  = {GPIOC, 6};  pin_map[8]  = {GPIOA, 5};
    pin_map[9]  = {GPIOA, 2};  pin_map[10] = {GPIOA, 0};
    pin_map[11] = {GPIOC, 3};  pin_map[12] = {GPIOC, 2};
    pin_map[13] = {GPIOC, 1};  pin_map[14] = {GPIOC, 0};
    pin_map[15] = {GPIOC, 15}; pin_map[16] = {GPIOC, 14};
    pin_map[17] = {GPIOC, 13};
}

static const uint32_t PORTA_MASK = (3u<<(0*2))|(3u<<(2*2))|(3u<<(5*2))|(3u<<(8*2))|(3u<<(9*2));
static const uint32_t PORTB_MASK = (3u<<(12*2))|(3u<<(13*2))|(3u<<(14*2))|(3u<<(15*2));
static const uint32_t PORTC_MASK = (3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|
                                    (3u<<(6*2))|(3u<<(13*2))|(3u<<(14*2))|(3u<<(15*2));

static inline void SetAllHiZ() {
    GPIOA->MODER &= ~PORTA_MASK;
    GPIOB->MODER &= ~PORTB_MASK;
    GPIOC->MODER &= ~PORTC_MASK;
}

static void ConfigureAllPinsAsOutput() {
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    uint16_t pA[] = {GPIO_PIN_0, GPIO_PIN_2, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_9};
    uint16_t pB[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
    uint16_t pC[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
                     GPIO_PIN_6, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
    for (auto pin : pA) { g.Pin = pin; HAL_GPIO_Init(GPIOA, &g); }
    for (auto pin : pB) { g.Pin = pin; HAL_GPIO_Init(GPIOB, &g); }
    for (auto pin : pC) { g.Pin = pin; HAL_GPIO_Init(GPIOC, &g); }
}

static inline void LightLED(uint8_t idx) {
    int8_t a = anode_map[idx];
    int8_t c = cathode_map[idx];
    if (a < 1 || c < 1) return;
    PinMapping& am = pin_map[a];
    PinMapping& cm = pin_map[c];
    SetAllHiZ();
    am.port->MODER |= (1u << (am.pin_pos * 2));
    cm.port->MODER |= (1u << (cm.pin_pos * 2));
    am.port->BSRR   = (1u << am.pin_pos);
    cm.port->BSRR   = (1u << (cm.pin_pos + 16));
}

static uint8_t fb[2][256];
static volatile uint8_t fb_display = 0; 
static volatile uint8_t fb_write   = 1;  

static inline void FB_Flip() {
    fb_display ^= 1;
    fb_write   ^= 1;
}

static inline void FB_ClearWrite() {
    memset(fb[fb_write], 0, 256);
}

static void FB_SetPixel(float sx, float sy) {
    int x = (int)roundf(sx);
    int y = (int)roundf(sy);
    if (x == 0) x = (sx >= 0.0f) ? 1 : -1;
    if (y == 0) y = (sy >= 0.0f) ? 1 : -1;
    if (x < -8 || x > 8 || y < -8 || y > 8) return;
    int col = (x < 0) ? (x + 8) : (x + 7);
    int row = (y > 0) ? (8 - y) : (-y + 7);
    int idx  = row * 16 + col;
    if (idx >= 0 && idx < 256) fb[fb_write][idx] = 1;
}

static volatile uint16_t mux_led = 0;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM2) return;

    SetAllHiZ();

    uint16_t start = mux_led;
    do {
        mux_led = (mux_led + 1) & 0xFF;
        if (fb[fb_display][mux_led]) {
            LightLED((uint8_t)mux_led);
            return;
        }
    } while (mux_led != start);
}

// Fluid Simulation
struct Particle { float x, y, vx, vy; };
static Particle particles[NUM_PARTICLES];

static void InitParticles() {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        float angle = ((float)i / NUM_PARTICLES) * 2.0f * (float)M_PI;
        float r = SIM_RADIUS * 0.5f * sqrtf((float)(i + 1) / NUM_PARTICLES);
        particles[i].x  =  r * cosf(angle);
        particles[i].y  = -SIM_RADIUS * 0.4f + r * 0.4f * sinf(angle);
        particles[i].vx = 0.0f;
        particles[i].vy = 0.0f;
    }
}

static void SimStep(float gx, float gy) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles[i].vx += gx * DT;
        particles[i].vy += gy * DT;
        particles[i].x  += particles[i].vx * DT;
        particles[i].y  += particles[i].vy * DT;
    }

    // Wall behavior
    for (int i = 0; i < NUM_PARTICLES; i++) {
        float dx   = particles[i].x;
        float dy   = particles[i].y;
        float dist = sqrtf(dx*dx + dy*dy);
        if (dist > SIM_RADIUS && dist > 0.001f) {
            float nx    = dx / dist;
            float ny    = dy / dist;
            float depth = dist - SIM_RADIUS;
            particles[i].x -= nx * depth;
            particles[i].y -= ny * depth;
            float dot = particles[i].vx * nx + particles[i].vy * ny;
            if (dot > 0.0f) {
                particles[i].vx -= RESTITUTION * dot * nx;
                particles[i].vy -= RESTITUTION * dot * ny;
                particles[i].vx *= DAMPING;
                particles[i].vy *= DAMPING;
            }
        }
    }

    // Particle to particle behavior
    for (int i = 0; i < NUM_PARTICLES; i++) {
        for (int j = i + 1; j < NUM_PARTICLES; j++) {
            float dx   = particles[i].x - particles[j].x;
            float dy   = particles[i].y - particles[j].y;
            float d2   = dx*dx + dy*dy;
            float min2 = PARTICLE_REPULSE * PARTICLE_REPULSE;
            if (d2 < min2 && d2 > 0.0001f) {
                float d = sqrtf(d2);
                float s = (PARTICLE_REPULSE - d) / d * 0.5f;
                particles[i].x += dx * s; particles[i].y += dy * s;
                particles[j].x -= dx * s; particles[j].y -= dy * s;
            }
        }
    }
}

// Accelerometer Functions
static volatile uint8_t device_id_ad = 0;

static void ADXL363_Init() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(10);
    uint8_t tx[3] = {0x0B, 0x00, 0x00}, rx[3] = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    device_id_ad = rx[2];
    if (device_id_ad == 0xAD) {
        uint8_t pw[3] = {0x0A, 0x2D, 0x02};
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, pw, 3, 100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(10);
    }
}

static void ADXL363_ReadGravity(float* gx_out, float* gy_out) {
    if (device_id_ad != 0xAD) {
        *gx_out = 0.0f;
        *gy_out = -GRAVITY_STRENGTH;
        return;
    }
    uint8_t tx[8] = {0x0B, 0x0E, 0,0,0,0,0,0}, rx[8] = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 8, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    int16_t raw_x = (int16_t)((rx[3] << 8) | rx[2]);
    int16_t raw_y = (int16_t)((rx[5] << 8) | rx[4]);
    float ax = raw_x / 1000.0f;
    float ay = raw_y / 1000.0f;
    float mag = sqrtf(ax*ax + ay*ay);
    if (mag > 1.0f) { ax /= mag; ay /= mag; }
    *gx_out =  ax * GRAVITY_STRENGTH;
    *gy_out = -ay * GRAVITY_STRENGTH;
}

// Sleep Wake Function
static bool CheckActivity(float gx, float gy, float prev_gx, float prev_gy) {
    float dx = gx - prev_gx;
    float dy = gy - prev_gy;
    return (dx*dx + dy*dy) > (ACTIVITY_THRESHOLD * ACTIVITY_THRESHOLD);
}

static void GoToSleep() {
    is_sleeping = true;
    HAL_TIM_Base_Stop_IT(&htim2);
    SetAllHiZ();
}

static void WakeUp(float gx, float gy) {
    is_sleeping = false;
    FB_ClearWrite();
    for (int i = 0; i < NUM_PARTICLES; i++) FB_SetPixel(particles[i].x, particles[i].y);
    FB_Flip();
    mux_led = 0;
    HAL_TIM_Base_Start_IT(&htim2);
}

// Main
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_ICACHE_Init();
    MX_TIM2_Init();

#ifdef PWR_UCPDR_UCPD_DBDIS
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->UCPDR |= PWR_UCPDR_UCPD_DBDIS;
#endif

    InitPinMap();
    ConfigureAllPinsAsOutput();
    ADXL363_Init();
    InitParticles();

    FB_ClearWrite();
    for (int i = 0; i < NUM_PARTICLES; i++) FB_SetPixel(particles[i].x, particles[i].y);
    FB_Flip();

    HAL_TIM_Base_Start_IT(&htim2);

    float gx = 0.0f, gy = -GRAVITY_STRENGTH;
    int accel_counter   = 0;
    int physics_counter = 0;
    int inactivity_count = 0;

    while (1) {
        static uint32_t last_tick = 0;
        while ((HAL_GetTick() - last_tick) < 16);
        last_tick = HAL_GetTick();

        if (!is_sleeping) {
            physics_counter++;
            if (physics_counter >= PHYSICS_STEPS_PER_PASS) {
                physics_counter = 0;
                SimStep(gx, gy);
                FB_ClearWrite();
                for (int i = 0; i < NUM_PARTICLES; i++) FB_SetPixel(particles[i].x, particles[i].y);
                FB_Flip();
            }
        }

        accel_counter++;
        if (accel_counter >= ACCEL_READ_INTERVAL) {
            accel_counter = 0;

            float prev_gx = gx, prev_gy = gy;
            ADXL363_ReadGravity(&gx, &gy);

            if (is_sleeping) {
                if (CheckActivity(gx, gy, prev_gx, prev_gy)) {
                    inactivity_count = 0;
                    WakeUp(gx, gy);
                }
            } else {
                if (CheckActivity(gx, gy, prev_gx, prev_gy)) {
                    inactivity_count = 0;
                } else {
                    inactivity_count++;
                    if (inactivity_count >= INACTIVITY_COUNT_MAX) {
                        inactivity_count = 0;
                        GoToSleep();
                    }
                }
            }
        }
    }
}

// System
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_4;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLMBOOST       = RCC_PLLMBOOST_DIV1;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 80;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLLVCIRANGE_0;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                       RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

extern "C" void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}