/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "mc_api.h"
#include "mc_config.h"
#include "r_divider_bus_voltage_sensor.h"
#include "ntc_temperature_sensor.h"
#include "ics_g4xx_pwm_curr_fdbk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CORDIC_HandleTypeDef hcordic;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* 觀測時間窗：16kHz 的 PWM 用 20~50ms 都很夠；我給 50ms 保守 */
#define UVW_SELFTEST_WINDOW_MS  (20u)

/* PASS 後 toggle 週期 */
#define UVW_TOGGLE_MS           (100u)

/* 用 GPIOB 的 PB3/PB4/PB5 當 test pin */
#define U_TEST_PORT  GPIOB
#define U_TEST_PIN   GPIO_PIN_3   // PB3
#define V_TEST_PORT  GPIOB
#define V_TEST_PIN   GPIO_PIN_4   // PB4
#define W_TEST_PORT  GPIOB
#define W_TEST_PIN   GPIO_PIN_5   // PB5
// for Isns check
#define ISNS_U_CHECK_PORT        GPIOB
#define ISNS_U_CHECK_PIN         GPIO_PIN_15   // IsnsU -> PB15

#define ISNS_V_CHECK_PORT      GPIOB
#define ISNS_V_CHECK_PIN       GPIO_PIN_14   // IsnsV -> PB14

#define ISNS_CHECK_TOGGLE_MS   100u

// bidirectional + 3.3V，0A 時理論中心點約 1.65V
#define ISNS_VOLTAGE_MIN       1.45f
#define ISNS_VOLTAGE_MAX       1.85f

/* Calibration Constants for VBUS derived from empirical data */
#define VBUS_CALIB_GAIN   0.986969f
#define VBUS_CALIB_OFFSET 0.520000f
/* Threshold to mask out USB backfeeding noise (e.g., voltages under 3.5V are treated as 0V) */
#define VBUS_NOISE_DEADBAND_V 3.5f

#define NTC_TABLE_SIZE 331U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//for Tsns
typedef struct
{
    float temp_c;
    uint16_t adc;
} ntc_lut_t;

const ntc_lut_t NTC_LUT[NTC_TABLE_SIZE] = {
    { -40.0f,   848 },
    { -39.5f,   880 },
    { -39.0f,   912 },
    { -38.5f,   944 },
    { -38.0f,   976 },
    { -37.5f,  1008 },
    { -37.0f,  1040 },
    { -36.5f,  1072 },
    { -36.0f,  1120 },
    { -35.5f,  1152 },
    { -35.0f,  1184 },
    { -34.5f,  1232 },
    { -34.0f,  1280 },
    { -33.5f,  1312 },
    { -33.0f,  1360 },
    { -32.5f,  1408 },
    { -32.0f,  1456 },
    { -31.5f,  1504 },
    { -31.0f,  1552 },
    { -30.5f,  1600 },
    { -30.0f,  1664 },
    { -29.5f,  1712 },
    { -29.0f,  1760 },
    { -28.5f,  1824 },
    { -28.0f,  1888 },
    { -27.5f,  1936 },
    { -27.0f,  2000 },
    { -26.5f,  2064 },
    { -26.0f,  2128 },
    { -25.5f,  2192 },
    { -25.0f,  2272 },
    { -24.5f,  2336 },
    { -24.0f,  2416 },
    { -23.5f,  2480 },
    { -23.0f,  2560 },
    { -22.5f,  2640 },
    { -22.0f,  2720 },
    { -21.5f,  2800 },
    { -21.0f,  2880 },
    { -20.5f,  2960 },
    { -20.0f,  3056 },
    { -19.5f,  3136 },
    { -19.0f,  3232 },
    { -18.5f,  3328 },
    { -18.0f,  3424 },
    { -17.5f,  3520 },
    { -17.0f,  3616 },
    { -16.5f,  3728 },
    { -16.0f,  3824 },
    { -15.5f,  3936 },
    { -15.0f,  4048 },
    { -14.5f,  4160 },
    { -14.0f,  4272 },
    { -13.5f,  4400 },
    { -13.0f,  4512 },
    { -12.5f,  4640 },
    { -12.0f,  4768 },
    { -11.5f,  4896 },
    { -11.0f,  5024 },
    { -10.5f,  5152 },
    { -10.0f,  5280 },
    {  -9.5f,  5424 },
    {  -9.0f,  5568 },
    {  -8.5f,  5712 },
    {  -8.0f,  5856 },
    {  -7.5f,  6000 },
    {  -7.0f,  6160 },
    {  -6.5f,  6320 },
    {  -6.0f,  6464 },
    {  -5.5f,  6624 },
    {  -5.0f,  6800 },
    {  -4.5f,  6960 },
    {  -4.0f,  7136 },
    {  -3.5f,  7312 },
    {  -3.0f,  7472 },
    {  -2.5f,  7664 },
    {  -2.0f,  7840 },
    {  -1.5f,  8032 },
    {  -1.0f,  8208 },
    {  -0.5f,  8400 },
    {   0.0f,  8592 },
    {   0.5f,  8800 },
    {   1.0f,  8992 },
    {   1.5f,  9200 },
    {   2.0f,  9408 },
    {   2.5f,  9616 },
    {   3.0f,  9824 },
    {   3.5f, 10032 },
    {   4.0f, 10256 },
    {   4.5f, 10480 },
    {   5.0f, 10704 },
    {   5.5f, 10928 },
    {   6.0f, 11152 },
    {   6.5f, 11392 },
    {   7.0f, 11632 },
    {   7.5f, 11872 },
    {   8.0f, 12112 },
    {   8.5f, 12352 },
    {   9.0f, 12608 },
    {   9.5f, 12848 },
    {  10.0f, 13104 },
    {  10.5f, 13360 },
    {  11.0f, 13616 },
    {  11.5f, 13888 },
    {  12.0f, 14144 },
    {  12.5f, 14416 },
    {  13.0f, 14688 },
    {  13.5f, 14960 },
    {  14.0f, 15232 },
    {  14.5f, 15504 },
    {  15.0f, 15792 },
    {  15.5f, 16080 },
    {  16.0f, 16352 },
    {  16.5f, 16640 },
    {  17.0f, 16944 },
    {  17.5f, 17232 },
    {  18.0f, 17520 },
    {  18.5f, 17824 },
    {  19.0f, 18112 },
    {  19.5f, 18416 },
    {  20.0f, 18720 },
    {  20.5f, 19024 },
    {  21.0f, 19328 },
    {  21.5f, 19632 },
    {  22.0f, 19952 },
    {  22.5f, 20256 },
    {  23.0f, 20576 },
    {  23.5f, 20880 },
    {  24.0f, 21200 },
    {  24.5f, 21520 },
    {  25.0f, 21840 },
    {  25.5f, 22160 },
    {  26.0f, 22480 },
    {  26.5f, 22800 },
    {  27.0f, 23120 },
    {  27.5f, 23456 },
    {  28.0f, 23776 },
    {  28.5f, 24096 },
    {  29.0f, 24432 },
    {  29.5f, 24752 },
    {  30.0f, 25088 },
    {  30.5f, 25408 },
    {  31.0f, 25744 },
    {  31.5f, 26080 },
    {  32.0f, 26400 },
    {  32.5f, 26736 },
    {  33.0f, 27056 },
    {  33.5f, 27392 },
    {  34.0f, 27728 },
    {  34.5f, 28048 },
    {  35.0f, 28384 },
    {  35.5f, 28720 },
    {  36.0f, 29040 },
    {  36.5f, 29376 },
    {  37.0f, 29712 },
    {  37.5f, 30032 },
    {  38.0f, 30368 },
    {  38.5f, 30688 },
    {  39.0f, 31024 },
    {  39.5f, 31344 },
    {  40.0f, 31664 },
    {  40.5f, 32000 },
    {  41.0f, 32320 },
    {  41.5f, 32640 },
    {  42.0f, 32960 },
    {  42.5f, 33280 },
    {  43.0f, 33600 },
    {  43.5f, 33920 },
    {  44.0f, 34240 },
    {  44.5f, 34560 },
    {  45.0f, 34880 },
    {  45.5f, 35184 },
    {  46.0f, 35504 },
    {  46.5f, 35808 },
    {  47.0f, 36112 },
    {  47.5f, 36432 },
    {  48.0f, 36736 },
    {  48.5f, 37040 },
    {  49.0f, 37344 },
    {  49.5f, 37632 },
    {  50.0f, 37936 },
    {  50.5f, 38240 },
    {  51.0f, 38528 },
    {  51.5f, 38832 },
    {  52.0f, 39120 },
    {  52.5f, 39408 },
    {  53.0f, 39696 },
    {  53.5f, 39984 },
    {  54.0f, 40256 },
    {  54.5f, 40544 },
    {  55.0f, 40832 },
    {  55.5f, 41104 },
    {  56.0f, 41376 },
    {  56.5f, 41648 },
    {  57.0f, 41920 },
    {  57.5f, 42192 },
    {  58.0f, 42464 },
    {  58.5f, 42720 },
    {  59.0f, 42992 },
    {  59.5f, 43248 },
    {  60.0f, 43504 },
    {  60.5f, 43760 },
    {  61.0f, 44016 },
    {  61.5f, 44272 },
    {  62.0f, 44512 },
    {  62.5f, 44752 },
    {  63.0f, 45008 },
    {  63.5f, 45248 },
    {  64.0f, 45488 },
    {  64.5f, 45728 },
    {  65.0f, 45952 },
    {  65.5f, 46192 },
    {  66.0f, 46416 },
    {  66.5f, 46656 },
    {  67.0f, 46880 },
    {  67.5f, 47104 },
    {  68.0f, 47328 },
    {  68.5f, 47536 },
    {  69.0f, 47760 },
    {  69.5f, 47968 },
    {  70.0f, 48176 },
    {  70.5f, 48400 },
    {  71.0f, 48608 },
    {  71.5f, 48800 },
    {  72.0f, 49008 },
    {  72.5f, 49216 },
    {  73.0f, 49408 },
    {  73.5f, 49600 },
    {  74.0f, 49808 },
    {  74.5f, 50000 },
    {  75.0f, 50176 },
    {  75.5f, 50368 },
    {  76.0f, 50560 },
    {  76.5f, 50736 },
    {  77.0f, 50928 },
    {  77.5f, 51104 },
    {  78.0f, 51280 },
    {  78.5f, 51456 },
    {  79.0f, 51632 },
    {  79.5f, 51792 },
    {  80.0f, 51968 },
    {  80.5f, 52128 },
    {  81.0f, 52304 },
    {  81.5f, 52464 },
    {  82.0f, 52624 },
    {  82.5f, 52784 },
    {  83.0f, 52944 },
    {  83.5f, 53088 },
    {  84.0f, 53248 },
    {  84.5f, 53408 },
    {  85.0f, 53552 },
    {  85.5f, 53696 },
    {  86.0f, 53840 },
    {  86.5f, 53984 },
    {  87.0f, 54128 },
    {  87.5f, 54272 },
    {  88.0f, 54416 },
    {  88.5f, 54544 },
    {  89.0f, 54688 },
    {  89.5f, 54816 },
    {  90.0f, 54944 },
    {  90.5f, 55072 },
    {  91.0f, 55200 },
    {  91.5f, 55328 },
    {  92.0f, 55456 },
    {  92.5f, 55584 },
    {  93.0f, 55712 },
    {  93.5f, 55824 },
    {  94.0f, 55952 },
    {  94.5f, 56064 },
    {  95.0f, 56176 },
    {  95.5f, 56288 },
    {  96.0f, 56400 },
    {  96.5f, 56512 },
    {  97.0f, 56624 },
    {  97.5f, 56736 },
    {  98.0f, 56848 },
    {  98.5f, 56944 },
    {  99.0f, 57056 },
    {  99.5f, 57152 },
    { 100.0f, 57264 },
    { 100.5f, 57360 },
    { 101.0f, 57456 },
    { 101.5f, 57552 },
    { 102.0f, 57648 },
    { 102.5f, 57744 },
    { 103.0f, 57840 },
    { 103.5f, 57936 },
    { 104.0f, 58032 },
    { 104.5f, 58112 },
    { 105.0f, 58208 },
    { 105.5f, 58288 },
    { 106.0f, 58384 },
    { 106.5f, 58464 },
    { 107.0f, 58544 },
    { 107.5f, 58640 },
    { 108.0f, 58720 },
    { 108.5f, 58800 },
    { 109.0f, 58880 },
    { 109.5f, 58960 },
    { 110.0f, 59040 },
    { 110.5f, 59120 },
    { 111.0f, 59184 },
    { 111.5f, 59264 },
    { 112.0f, 59344 },
    { 112.5f, 59408 },
    { 113.0f, 59488 },
    { 113.5f, 59552 },
    { 114.0f, 59632 },
    { 114.5f, 59696 },
    { 115.0f, 59760 },
    { 115.5f, 59824 },
    { 116.0f, 59904 },
    { 116.5f, 59968 },
    { 117.0f, 60032 },
    { 117.5f, 60096 },
    { 118.0f, 60160 },
    { 118.5f, 60224 },
    { 119.0f, 60272 },
    { 119.5f, 60336 },
    { 120.0f, 60400 },
    { 120.5f, 60464 },
    { 121.0f, 60512 },
    { 121.5f, 60576 },
    { 122.0f, 60624 },
    { 122.5f, 60688 },
    { 123.0f, 60736 },
    { 123.5f, 60800 },
    { 124.0f, 60848 },
    { 124.5f, 60912 },
    { 125.0f, 60960 },
};

/* Function to calculate temperature using linear interpolation based on LUT */
float Get_Temperature_From_Table(uint16_t adc_raw_16bit)
{
    uint16_t adc_value = adc_raw_16bit;

    if (adc_value <= NTC_LUT[0].adc)
        return NTC_LUT[0].temp_c;

    if (adc_value >= NTC_LUT[NTC_TABLE_SIZE - 1].adc)
        return NTC_LUT[NTC_TABLE_SIZE - 1].temp_c;

    for (uint16_t i = 0; i < (NTC_TABLE_SIZE - 1U); i++)
    {
        uint16_t adc_lo = NTC_LUT[i].adc;
        uint16_t adc_hi = NTC_LUT[i + 1U].adc;

        if ((adc_value >= adc_lo) && (adc_value <= adc_hi))
        {
            float temp_lo = NTC_LUT[i].temp_c;
            float temp_hi = NTC_LUT[i + 1U].temp_c;

            if (adc_hi == adc_lo)
                return temp_lo;

            float ratio = ((float)adc_value - (float)adc_lo) /
                          ((float)adc_hi - (float)adc_lo);

            return temp_lo + (ratio * (temp_hi - temp_lo));
        }
    }

    return -999.0f;
}

float Get_Temperature_From_Table_Nearest(uint16_t adc_raw_16bit)
{
    uint16_t adc_value = adc_raw_16bit;

    if (adc_value <= NTC_LUT[0].adc)
        return NTC_LUT[0].temp_c;

    if (adc_value >= NTC_LUT[NTC_TABLE_SIZE - 1].adc)
        return NTC_LUT[NTC_TABLE_SIZE - 1].temp_c;

    for (uint16_t i = 0; i < (NTC_TABLE_SIZE - 1U); i++)
    {
        uint16_t adc_lo = NTC_LUT[i].adc;
        uint16_t adc_hi = NTC_LUT[i + 1U].adc;

        if ((adc_value >= adc_lo) && (adc_value <= adc_hi))
        {
            uint16_t diff_lo = adc_value - adc_lo;
            uint16_t diff_hi = adc_hi - adc_value;

            if (diff_lo <= diff_hi)
                return NTC_LUT[i].temp_c;
            else
                return NTC_LUT[i + 1U].temp_c;
        }
    }

    return -999.0f;
}

/* Function to get calibrated VBUS voltage in floating point */
float Get_Calibrated_VBUS_V(void)
{
    float raw_calculated_voltage;
    float final_calibrated_voltage;
    uint16_t raw_voltage_d;

    /* 1. Retrieve Raw ADC Data for VBUS */
    raw_voltage_d = VBS_GetAvBusVoltage_d((BusVoltageSensor_Handle_t *)&BusVoltageSensor_M1);

    /* 2. Calculate the initial floating-point voltage using ST's default conversion factor */
    raw_calculated_voltage = (float)raw_voltage_d * ((BusVoltageSensor_Handle_t *)&BusVoltageSensor_M1)->ConversionFactor / 65536.0f;

    /* 3. Apply secondary linear calibration formula: y = mx + c */
    final_calibrated_voltage = (raw_calculated_voltage * VBUS_CALIB_GAIN) + VBUS_CALIB_OFFSET;

    /* 4. Apply Deadband: Mask out USB backfeed voltage and negative noise */
    if (final_calibrated_voltage < VBUS_NOISE_DEADBAND_V)
    {
        final_calibrated_voltage = 0.0f;
    }

    return final_calibrated_voltage;
}


static inline void UVW_TestPins_SetLow(void)
{
  HAL_GPIO_WritePin(U_TEST_PORT, U_TEST_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V_TEST_PORT, V_TEST_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(W_TEST_PORT, W_TEST_PIN, GPIO_PIN_RESET);
}

void TIM1_UVW_SelfTest_Task_SDK(void)
{
    static uint32_t s_last_toggle = 0u;
    uint32_t now = HAL_GetTick();

    /* Get the motor state machine status via SDK */
    MCI_State_t mcState = MC_GetSTMStateMotor1();

    /* Check PWM pulse width only when in starting or running state */
    if (mcState == START || mcState == RUN)
    {
        if ((now - s_last_toggle) >= UVW_TOGGLE_MS)
        {
            /* Read-only access to TIM1 Compare registers, without interfering with FOC */
            bool pwm_u_active = (TIM1->CCR1 > 0);
            bool pwm_v_active = (TIM1->CCR2 > 0);
            bool pwm_w_active = (TIM1->CCR3 > 0);

            if (pwm_u_active) HAL_GPIO_TogglePin(U_TEST_PORT, U_TEST_PIN);
            else HAL_GPIO_WritePin(U_TEST_PORT, U_TEST_PIN, GPIO_PIN_RESET);

            if (pwm_v_active) HAL_GPIO_TogglePin(V_TEST_PORT, V_TEST_PIN);
            else HAL_GPIO_WritePin(V_TEST_PORT, V_TEST_PIN, GPIO_PIN_RESET);

            if (pwm_w_active) HAL_GPIO_TogglePin(W_TEST_PORT, W_TEST_PIN);
            else HAL_GPIO_WritePin(W_TEST_PORT, W_TEST_PIN, GPIO_PIN_RESET);

            s_last_toggle = now;
        }
    }
    else
    {
        UVW_TestPins_SetLow();
    }
}

void Vdc_check_fun_SDK(void)
{
    static uint32_t s_last_vdc_tick = 0u;
    static uint32_t s_vdc_period_ms = 10u;
    static bool s_is_vdc_ok = false; /* Hysteresis state flag */
    uint32_t now = HAL_GetTick();

    if ((now - s_last_vdc_tick) < s_vdc_period_ms) return;
    s_last_vdc_tick = now;

    /* Get the filtered bus voltage in Volts */
    float real_voltage_V = Get_Calibrated_VBUS_V();

    /* Hysteresis parameters for VBUS (Target: 2V ~ 50V) */
    const uint16_t VDC_LOW_FAIL       = 2;
    const uint16_t VDC_LOW_RECOVER    = 3;  /* Require 3V to recover from low voltage */
    const uint16_t VDC_HIGH_FAIL      = 50;
    const uint16_t VDC_HIGH_RECOVER   = 48; /* Require 48V to recover from over voltage */

    /* Hysteresis logic evaluation */
    if (!s_is_vdc_ok)
    {
        /* Turn ON condition: Voltage must be safely inside the inner window */
        if ((real_voltage_V >= VDC_LOW_RECOVER) && (real_voltage_V <= VDC_HIGH_RECOVER))
        {
            s_is_vdc_ok = true;
        }
    }
    else
    {
        /* Turn OFF condition: Voltage hits the absolute outer limits */
        if ((real_voltage_V < VDC_LOW_FAIL) || (real_voltage_V > VDC_HIGH_FAIL))
        {
            s_is_vdc_ok = false;
        }
    }

    /* Output evaluation based on state flag */
    if (s_is_vdc_ok)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        s_vdc_period_ms = 100u;   /* Normal state */
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        s_vdc_period_ms = 100u;  /* Abnormal state */
    }
}

void TSNS_Check_SDK(void)
{
    static uint32_t s_last_tsns_tick = 0;
    static bool s_is_tsns_ok = false; /* Hysteresis state flag */
    uint32_t now = HAL_GetTick();

    if ((now - s_last_tsns_tick) < 100u) return;
    s_last_tsns_tick = now;

    /* Get the raw 16-bit ADC value from SDK */
	uint16_t raw_temp_d = NTC_GetAvTemp_d(&TempSensor_M1);

    /* Pure lookup table only, no linear interpolation */
    float table_temp_C = Get_Temperature_From_Table_Nearest(raw_temp_d);

    /* Hysteresis parameters for TSNS (Target: -10 C ~ 120 C) */
    const int16_t TSNS_LOW_FAIL     = -10;
    const int16_t TSNS_LOW_RECOVER  = -5;  /* Require -5 C to recover from cold */
    const int16_t TSNS_HIGH_FAIL    = 120;
    const int16_t TSNS_HIGH_RECOVER = 110; /* Require 110 C to recover from overheat */

    /* Hysteresis logic evaluation */
    if (!s_is_tsns_ok)
    {
        /* Turn ON condition: Temperature must be safely inside the inner window */
        if ((table_temp_C >= TSNS_LOW_RECOVER) && (table_temp_C <= TSNS_HIGH_RECOVER))
        {
            s_is_tsns_ok = true;
        }
    }
    else
    {
        /* Turn OFF condition: Temperature hits the absolute outer limits */
        if ((table_temp_C <= TSNS_LOW_FAIL) || (table_temp_C >= TSNS_HIGH_FAIL))
        {
            s_is_tsns_ok = false;
        }
    }

    if (s_is_tsns_ok)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    }
}

void Isns_Check_SDK(void)
{
    static uint32_t s_isns_last_toggle = 0u;
    static bool s_is_isns_u_ok = false; /* Hysteresis state for Phase U */
    static bool s_is_isns_v_ok = false; /* Hysteresis state for Phase V */

    /* If the motor has already started, stop testing the pins */
    MCI_State_t mcState = MC_GetSTMStateMotor1();
    if (mcState == START || mcState == RUN)
    {
        HAL_GPIO_WritePin(ISNS_U_CHECK_PORT, ISNS_U_CHECK_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ISNS_V_CHECK_PORT, ISNS_V_CHECK_PIN, GPIO_PIN_RESET);
        return;
    }

    if ((HAL_GetTick() - s_isns_last_toggle) >= ISNS_CHECK_TOGGLE_MS)
    {
        PWMC_ICS_Handle_t *pIcsHandle = (PWMC_ICS_Handle_t *)pwmcHandle[0];

        /* Read the converted digital value (0~4095) */
        uint16_t raw_adc_A = (pIcsHandle->pParams_str->ADCx_1->JDR1) >> 4;
        uint16_t raw_adc_B = (pIcsHandle->pParams_str->ADCx_2->JDR1) >> 4;

        /* Hysteresis bounds for ISNS (Target: 1985 ~ 2110) */
        const uint16_t ISNS_LOWER_FAIL    = 1985;
        const uint16_t ISNS_LOWER_RECOVER = 2005; /* +20 margin to recover */
        const uint16_t ISNS_UPPER_FAIL    = 2110;
        const uint16_t ISNS_UPPER_RECOVER = 2090; /* -20 margin to recover */

        /* Validate hardware Isns Phase A with Hysteresis */
        if (!s_is_isns_u_ok)
        {
            if ((raw_adc_A >= ISNS_LOWER_RECOVER) && (raw_adc_A <= ISNS_UPPER_RECOVER)) s_is_isns_u_ok = true;
        }
        else
        {
            if ((raw_adc_A < ISNS_LOWER_FAIL) || (raw_adc_A > ISNS_UPPER_FAIL)) s_is_isns_u_ok = false;
        }

        if (s_is_isns_u_ok) HAL_GPIO_TogglePin(ISNS_U_CHECK_PORT, ISNS_U_CHECK_PIN);
        else HAL_GPIO_WritePin(ISNS_U_CHECK_PORT, ISNS_U_CHECK_PIN, GPIO_PIN_RESET);

        /* Validate hardware Isns Phase B with Hysteresis */
        if (!s_is_isns_v_ok)
        {
            if ((raw_adc_B >= ISNS_LOWER_RECOVER) && (raw_adc_B <= ISNS_UPPER_RECOVER)) s_is_isns_v_ok = true;
        }
        else
        {
            if ((raw_adc_B < ISNS_LOWER_FAIL) || (raw_adc_B > ISNS_UPPER_FAIL)) s_is_isns_v_ok = false;
        }

        if (s_is_isns_v_ok) HAL_GPIO_TogglePin(ISNS_V_CHECK_PORT, ISNS_V_CHECK_PIN);
        else HAL_GPIO_WritePin(ISNS_V_CHECK_PORT, ISNS_V_CHECK_PIN, GPIO_PIN_RESET);

        s_isns_last_toggle = HAL_GetTick();
    }
}

void GaN_Safe_AutoStart_Task(void)
{
    static uint32_t s_stable_voltage_start_time = 0;
    static bool s_is_voltage_stable = false;

    /* This flag allows clearing severe faults after a manual power cycle */
    static bool s_allow_reset_after_power_cycle = false;

    MCI_State_t mcState = MC_GetSTMStateMotor1();
    float real_voltage_V = Get_Calibrated_VBUS_V();

    /* 1. Reset Detection: Detect if 12V was removed */
    if (real_voltage_V < 4)
    {
        s_is_voltage_stable = false;
        s_allow_reset_after_power_cycle = true;
    }

    /* 2. Safe fault reset logic (to protect GaN devices) */
    if (mcState == FAULT_OVER)
    {
        uint16_t pastFaults = MC_GetOccurredFaultsMotor1();

        /* Clear fault IF it was Under-Voltage OR the user performed a power cycle */
        if (pastFaults == MC_UNDER_VOLT || s_allow_reset_after_power_cycle)
        {
            MC_AcknowledgeFaultMotor1();
            s_allow_reset_after_power_cycle = false;
        }
        else
        {
            HAL_GPIO_WritePin(ISNS_V_CHECK_PORT, ISNS_V_CHECK_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ISNS_U_CHECK_PORT, ISNS_U_CHECK_PIN, GPIO_PIN_SET);
        }
    }
    /* 3. Stabilization delay and safe start logic */
    else if (mcState == IDLE)
    {
        if (real_voltage_V >= 4 && real_voltage_V <= 50)
        {
            if (!s_is_voltage_stable)
            {
                s_stable_voltage_start_time = HAL_GetTick();
                s_is_voltage_stable = true;
            }
            else
            {
                if ((HAL_GetTick() - s_stable_voltage_start_time) > 500u)
                {
                    MC_StartMotor1();
                }
            }
        }
        else
        {
            s_is_voltage_stable = false;
        }
    }
    else
    {
        s_is_voltage_stable = false;
    }
}

// UART transmission setting
void UART3_Print(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void Debug_Print_Task(void)
{
    static uint32_t s_last_print = 0;
    uint32_t now = HAL_GetTick();

    // Print every 500ms to avoid flooding the UART port
    if ((now - s_last_print) < 500u)
        return;
    s_last_print = now;

    char msg[250];
    MCI_State_t mcState = MC_GetSTMStateMotor1();

    /* Retrieve Voltage Data (Physical + Raw ADC) */
    uint16_t raw_voltage_d  = VBS_GetAvBusVoltage_d((BusVoltageSensor_Handle_t *)&BusVoltageSensor_M1);
    /* Calculate true floating-point voltage using the raw value and conversion factor */
	/* Ensure the division is done using a floating-point number (65536.0) */
    float real_voltage_V = Get_Calibrated_VBUS_V();
    /* Temperature: pure lookup table only */
    uint16_t raw_temp_d = NTC_GetAvTemp_d(&TempSensor_M1);
    float table_temp_C = Get_Temperature_From_Table_Nearest(raw_temp_d);

    /* Retrieve Current Data (Raw ADC) */
    PWMC_ICS_Handle_t *pIcsHandle = (PWMC_ICS_Handle_t *)pwmcHandle[0];
    uint16_t raw_adc_A = (pIcsHandle->pParams_str->ADCx_1->JDR1) >> 4;
    uint16_t raw_adc_B = (pIcsHandle->pParams_str->ADCx_2->JDR1) >> 4;

    // Get PWM enable status
	uint8_t pwm_is_on = (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0;

    // Format the status message including RAW data for TSNS and VBUS
    snprintf(msg, sizeof(msg),
             "ST=%d, VBUS=%.2fV(RAW:%u), TSNS=%.1fC(RAW:%u), I_U=%u, I_V=%u, PWM=%d\r\n",
             mcState, real_voltage_V, raw_voltage_d, table_temp_C, raw_temp_d, raw_adc_A, raw_adc_B, pwm_is_on);

    UART3_Print(msg);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 TIM1_UVW_SelfTest_Task_SDK();
	 Vdc_check_fun_SDK();
	 Isns_Check_SDK();
	 TSNS_Check_SDK();
	 // Execute the safe auto-start and protection logic for GaN
	 GaN_Safe_AutoStart_Task();
	 Debug_Print_Task();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* Run hardware calibration for ADC1 before starting the motor control SDK */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  /* Run hardware calibration for ADC2 before starting the motor control SDK */
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 1;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ISNS_V_CHECK_Pin|ISNS_U_CHECK_Pin|U_TEST_PIN_Pin|V_TEST_PIN_Pin
                          |W_TEST_PIN_Pin|Vdc_check_Pin|Tsns_check_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ISNS_V_CHECK_Pin ISNS_U_CHECK_Pin U_TEST_PIN_Pin V_TEST_PIN_Pin
                           W_TEST_PIN_Pin Vdc_check_Pin Tsns_check_Pin */
  GPIO_InitStruct.Pin = ISNS_V_CHECK_Pin|ISNS_U_CHECK_Pin|U_TEST_PIN_Pin|V_TEST_PIN_Pin
                          |W_TEST_PIN_Pin|Vdc_check_Pin|Tsns_check_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
