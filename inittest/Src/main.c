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
//for Tsns
#define TSNS_ADC_MIN_VALID   50
#define TSNS_ADC_MAX_VALID   4040
#define TSNS_SAMPLE_COUNT    16

/* Calibration Constants for VBUS derived from empirical data */
#define VBUS_CALIB_GAIN   0.986969f
#define VBUS_CALIB_OFFSET 0.520000f
/* Threshold to mask out USB backfeeding noise (e.g., voltages under 3.5V are treated as 0V) */
#define VBUS_NOISE_DEADBAND_V 3.5f

#define NTC_TABLE_SIZE 34
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

/* Temperature points from -40 C to 125 C */
const int16_t NTC_TEMP_TABLE[NTC_TABLE_SIZE] = {
    -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40,
    45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125
};

/* Corresponding 12-bit ADC raw values from the newly provided CSV file */
const uint16_t NTC_ADC_TABLE[NTC_TABLE_SIZE] = {
    53, 74, 104, 142, 191, 253, 330, 425, 537, 669, 819, 987, 1170, 1365,
    1568, 1774, 1979, 2180, 2371, 2552, 2719, 2872, 3011, 3136, 3248, 3347,
    3434, 3511, 3579, 3638, 3690, 3735, 3775, 3810
};

/* Function to calculate temperature using linear interpolation based on LUT */
float Get_Temperature_From_Table(uint16_t adc_raw_16bit)
{
    /* Convert ST's 16-bit left-aligned ADC value back to actual 12-bit value */
    uint16_t adc_12bit = adc_raw_16bit >> 4;

    /* Handle out-of-bounds lower limit */
    if (adc_12bit <= NTC_ADC_TABLE[0]) return (float)NTC_TEMP_TABLE[0];

    /* Handle out-of-bounds upper limit */
    if (adc_12bit >= NTC_ADC_TABLE[NTC_TABLE_SIZE - 1]) return (float)NTC_TEMP_TABLE[NTC_TABLE_SIZE - 1];

    /* Perform linear interpolation */
    for (uint8_t i = 0; i < NTC_TABLE_SIZE - 1; i++)
    {
        if (adc_12bit >= NTC_ADC_TABLE[i] && adc_12bit <= NTC_ADC_TABLE[i + 1])
        {
            float adc_diff = (float)(NTC_ADC_TABLE[i + 1] - NTC_ADC_TABLE[i]);
            float temp_diff = (float)(NTC_TEMP_TABLE[i + 1] - NTC_TEMP_TABLE[i]);
            float ratio = (float)(adc_12bit - NTC_ADC_TABLE[i]) / adc_diff;
            return (float)NTC_TEMP_TABLE[i] + (ratio * temp_diff);
        }
    }
    return -99.0f;
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

	/* Calculate precise temperature using our LUT */
	float precise_temp_C = Get_Temperature_From_Table(raw_temp_d);

	/* Cast to int16_t to match your existing hysteresis logic type */
	int16_t temp_C = (int16_t)precise_temp_C;

    /* Hysteresis parameters for TSNS (Target: -10 C ~ 120 C) */
    const int16_t TSNS_LOW_FAIL     = -10;
    const int16_t TSNS_LOW_RECOVER  = -5;  /* Require -5 C to recover from cold */
    const int16_t TSNS_HIGH_FAIL    = 120;
    const int16_t TSNS_HIGH_RECOVER = 110; /* Require 110 C to recover from overheat */

    /* Hysteresis logic evaluation */
    if (!s_is_tsns_ok)
    {
        /* Turn ON condition: Temperature must be safely inside the inner window */
        if ((temp_C >= TSNS_LOW_RECOVER) && (temp_C <= TSNS_HIGH_RECOVER))
        {
            s_is_tsns_ok = true;
        }
    }
    else
    {
        /* Turn OFF condition: Temperature hits the absolute outer limits */
        if ((temp_C <= TSNS_LOW_FAIL) || (temp_C >= TSNS_HIGH_FAIL))
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
    /* Retrieve Temperature Data (Raw ADC + Precise Physical) */
	uint16_t raw_temp_d = NTC_GetAvTemp_d(&TempSensor_M1);
	float precise_temp_C = Get_Temperature_From_Table(raw_temp_d);
	int16_t temp_C = (int16_t)precise_temp_C; /* Keep as integer for existing snprintf format */
	int16_t temp_dec = (int16_t)(precise_temp_C * 10.0f) % 10;

	/* Ensure the decimal part is always positive for printing */
	if (temp_dec < 0)
	{
	    temp_dec = -temp_dec;
	}

    /* Retrieve Current Data (Raw ADC) */
    PWMC_ICS_Handle_t *pIcsHandle = (PWMC_ICS_Handle_t *)pwmcHandle[0];
    uint16_t raw_adc_A = (pIcsHandle->pParams_str->ADCx_1->JDR1) >> 4;
    uint16_t raw_adc_B = (pIcsHandle->pParams_str->ADCx_2->JDR1) >> 4;

    // Get PWM enable status
	uint8_t pwm_is_on = (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0;

    // Format the status message including RAW data for TSNS and VBUS
    snprintf(msg, sizeof(msg),
             "ST=%d, VBUS=%.2fV(RAW:%u), TSNS=%d.%dC(RAW:%u), I_U=%u, I_V=%u, PWM=%d\r\n",
             mcState, real_voltage_V, raw_voltage_d, temp_C, temp_dec, raw_temp_d, raw_adc_A, raw_adc_B, pwm_is_on);

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
//	 GaN_Safe_AutoStart_Task();
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
