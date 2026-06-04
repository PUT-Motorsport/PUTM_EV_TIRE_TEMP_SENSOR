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
#include "fdcan.h"
#include "i2c.h"
#include "lptim.h"
#include "stm32g0xx_hal_i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "PUTM_EV_CAN_LIBRARY/include/can_driver.hpp"
#include "PUTM_EV_CAN_LIBRARY/database/generated/PUTM_CAN_M.h"

//putm_ev_can :: CanDriver can_m;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#include "MLX90621_API.h"
//#include "MLX90621_I2C_Driver.h"

// Structure to hold values extracted from EEPROM
typedef struct {
    uint8_t VTH_L, VTH_H;
    uint8_t KT1_L, KT1_H;
    uint8_t KT2_L, KT2_H;
    uint8_t KT_scale; // Address 0xD2
} MLX_EEPROM_Ta_Coeffs;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MLX_EEPROM_SA    0x50  // Slave Address for EEPROM
#define MLX_I2C_TIMEOUT  1000  // 1 second timeout
#define MLX_RAM_SA       (0x60 << 1) // Slave Address 0x60 shifted for HAL
#define CMD_READ_RAM     0x02
#define I2C_TIMEOUT      100

static const float emissivity = 0.98f;
static const float tr = 15.0f;

static uint8_t mlx90621ToAverage[8] = {0};
static uint8_t eeMLX90621[256]; // The 256-byte EEPROM dump array
//static paramsMLX90621 mlx90621;
//static uint16_t mlx90621Frame[66];
//static float mlx90621To[64];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Melexis_Initialize(void);
uint16_t MLX90621_ReadPTAT(void);
uint16_t MLX90621_ReadCompensationPixel(void);
void MLX90621_ReadWholeFrame(uint16_t* frameData);
int16_t CheckSign(uint16_t value, uint16_t bit_length);
float Calculate_Ta(uint16_t ptat_data, MLX_EEPROM_Ta_Coeffs *eeprom, uint8_t config_res);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- Sensor Initialization ---
void Melexis_Initialize(void)
{
    // Wait at least 5ms after POR release
    HAL_Delay(5); 

    // 1. Read the whole EEPROM (SA = 0x50) using Mem_Read. 
    // It's exactly 256 bytes, so no endianness swapping is needed for the raw dump.
    HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(MLX_EEPROM_SA << 1), 0x00, I2C_MEMADD_SIZE_8BIT, eeMLX90621, 256, MLX_I2C_TIMEOUT);

    // 2. Write the oscillator trimming value (Command = 0x04, SA = 0x60)
    uint8_t trim_val = eeMLX90621[0xF7]; 
    uint8_t trim_payload[5];
    trim_payload[0] = 0x04;                        // Command
    trim_payload[1] = trim_val - 0xAA;             // LSByte check
    trim_payload[2] = trim_val;                    // LSByte
    trim_payload[3] = 0x00 - 0xAA;                 // MSByte check
    trim_payload[4] = 0x00;                        // MSByte
    
    HAL_I2C_Master_Transmit(&hi2c2, MLX_RAM_SA, trim_payload, 5, MLX_I2C_TIMEOUT);

    // 3. Write configuration register (Command = 0x03, SA = 0x60)
    uint8_t cfg_lsb = eeMLX90621[0xF5]; 
    uint8_t cfg_msb = eeMLX90621[0xF6]; 
    uint8_t cfg_payload[5];
    cfg_payload[0] = 0x03;                         // Command
    cfg_payload[1] = cfg_lsb - 0x55;               // LSByte check
    cfg_payload[2] = cfg_lsb;                      // LSbyte
    cfg_payload[3] = cfg_msb - 0x55;               // MSByte check
    cfg_payload[4] = cfg_msb;                      // MSbyte

    HAL_I2C_Master_Transmit(&hi2c2, MLX_RAM_SA, cfg_payload, 5, MLX_I2C_TIMEOUT);
}

// --- Read PTAT Data (Interrupt Mode with Repeated Start) ---
uint16_t MLX90621_ReadPTAT(void)
{
    uint8_t cmd[4] = {CMD_READ_RAM, 0x40, 0x00, 0x01};
    uint8_t rx_buf[2];
    
    // Wait until I2C bus is free
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    
    // Transmit command and KEEP the bus locked (No STOP condition)
    HAL_I2C_Master_Seq_Transmit_IT(&hi2c2, MLX_RAM_SA, cmd, 4, I2C_FIRST_FRAME);
    
    // Wait for the transmission to finish
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    
    // Repeated start, read data, and release the bus (STOP condition)
    HAL_I2C_Master_Seq_Receive_IT(&hi2c2, MLX_RAM_SA, rx_buf, 2, I2C_LAST_FRAME);
    
    // Wait for the reception to finish before processing the array
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    
    return (uint16_t)((rx_buf[1] << 8) | rx_buf[0]);
}

// --- Read Compensation Pixel Data (Interrupt Mode) ---
uint16_t MLX90621_ReadCompensationPixel(void)
{
    uint8_t cmd[4] = {CMD_READ_RAM, 0x41, 0x00, 0x01};
    uint8_t rx_buf[2];
    
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Seq_Transmit_IT(&hi2c2, MLX_RAM_SA, cmd, 4, I2C_FIRST_FRAME);
    
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Seq_Receive_IT(&hi2c2, MLX_RAM_SA, rx_buf, 2, I2C_LAST_FRAME);
    
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    
    return (uint16_t)((rx_buf[1] << 8) | rx_buf[0]);
}

// --- Read Whole IR Frame (Interrupt Mode) ---
void MLX90621_ReadWholeFrame(uint16_t* frameData)
{
    uint8_t cmd[4] = {CMD_READ_RAM, 0x00, 0x01, 0x40};
    uint8_t rx_buf[128]; // 64 pixels * 2 bytes each
    
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Seq_Transmit_IT(&hi2c2, MLX_RAM_SA, cmd, 4, I2C_FIRST_FRAME);
    
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Seq_Receive_IT(&hi2c2, MLX_RAM_SA, rx_buf, 128, I2C_LAST_FRAME);
    
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    
    for (int i = 0; i < 64; i++) {
        frameData[i] = (uint16_t)((rx_buf[i*2+1] << 8) | rx_buf[i*2]);
    }
}

// --- Check Sign for 2's Complement ---
int16_t CheckSign(uint16_t value, uint16_t bit_length) {
    uint16_t sign_bit = 1 << (bit_length - 1);
    if (value & sign_bit) {
        return (int16_t)(value - (1 << bit_length));
    }
    return (int16_t)value;
}

// --- Calculate Ta ---
float Calculate_Ta(uint16_t ptat_data, MLX_EEPROM_Ta_Coeffs *eeprom, uint8_t config_res) 
{
    uint8_t kt1_scale_bits = (eeprom->KT_scale & 0xF0) >> 4;
    uint8_t kt2_scale_bits = (eeprom->KT_scale & 0x0F);
    
    int16_t vth_raw = (eeprom->VTH_H << 8) | eeprom->VTH_L;
    int16_t kt1_raw = (eeprom->KT1_H << 8) | eeprom->KT1_L;
    int16_t kt2_raw = (eeprom->KT2_H << 8) | eeprom->KT2_L;
    
    float vth_25 = (float)vth_raw;
    float kt1 = (float)kt1_raw;
    float kt2 = (float)kt2_raw;
    
    // Correctly scale resolution according to datasheet
    float res_scale = pow(2, 3 - config_res); 
    
    vth_25 = vth_25 / res_scale;
    kt1 = kt1 / (pow(2, kt1_scale_bits) * res_scale);
    kt2 = kt2 / (pow(2, kt2_scale_bits + 10) * res_scale);
    
    float ptat = (float)ptat_data;
    
    float sqr_term = (kt1 * kt1) - (4.0f * kt2 * (vth_25 - ptat));
    float ta = (-kt1 + sqrt(sqr_term)) / (2.0f * kt2);
    ta += 25.0f;
    
    return ta;
}

void Calculate_To_All_Pixels(uint16_t *ir_frame_raw, float current_Ta, uint16_t comp_pixel_raw, uint8_t *eeprom, uint8_t config_res, float *pixel_temp_obj) 
{
    // 1. Calculate Resolution Scale
    double res_scale = pow(2.0, 3.0 - config_res); //

    // 2. Extract and Process Common EEPROM Calibration Constants
    int16_t a_common_raw = (int16_t)((eeprom[0xD1] << 8) | eeprom[0xD0]); //
    
    int16_t a_cp_raw = (int16_t)((eeprom[0xD4] << 8) | eeprom[0xD3]); //
    double a_cp = (double)a_cp_raw / res_scale; //
    
    int8_t b_cp_raw = (int8_t)eeprom[0xD5]; //
    
    uint16_t alpha_cp_raw = (uint16_t)((eeprom[0xD7] << 8) | eeprom[0xD6]); //
    
    int8_t tgc_raw = (int8_t)eeprom[0xD8]; //
    double tgc = (double)tgc_raw / 32.0; //
    
    uint8_t delta_a_scale = (eeprom[0xD9] & 0xF0) >> 4; //
    uint8_t b_i_scale = eeprom[0xD9] & 0x0F; //
    
    uint16_t alpha_0_raw = (uint16_t)((eeprom[0xE1] << 8) | eeprom[0xE0]); //
    uint8_t alpha_0_scale = eeprom[0xE2]; //
    uint8_t delta_alpha_scale = eeprom[0xE3]; //
    
    uint16_t emissivity_raw = (uint16_t)((eeprom[0xE5] << 8) | eeprom[0xE4]); //
    double emissivity = (double)emissivity_raw / 32768.0; //
    
    int16_t k_sta_raw = (int16_t)((eeprom[0xE7] << 8) | eeprom[0xE6]); //
    double k_sta = (double)k_sta_raw / 1048576.0; // 2^20
    
    int8_t k_s4_raw = (int8_t)eeprom[0xC4]; //
    uint8_t k_s_scale = eeprom[0xC0] & 0x0F; //
    double k_s4 = (double)k_s4_raw / pow(2.0, (double)(k_s_scale + 8)); //

    // Pre-calculate Compensation Pixel components
    double b_cp = (double)b_cp_raw / (pow(2.0, (double)b_i_scale) * res_scale); //
    double alpha_cp = (double)alpha_cp_raw / (pow(2.0, (double)alpha_0_scale) * res_scale); //
    double ta_minus_25 = (double)current_Ta - 25.0; //
    
    // Calculate Compensation Pixel Offset (V_IRcp_OffsetCompensated)
    int16_t v_cp = (int16_t)comp_pixel_raw; //
    double v_ircp_offset_comp = (double)v_cp - (a_cp + b_cp * ta_minus_25); //

    // Pre-calculate Ta_K^4 (Ambient Temperature in Kelvin to the 4th power)
    double ta_k = (double)current_Ta + 273.15; //
    double ta_k4 = pow(ta_k, 4.0); //

    // 3. Iterate over all 64 Pixels
    for (int i = 0; i < 64; i++) 
    {
        // Extract pixel-specific EEPROM data
        uint8_t delta_a_i = eeprom[i]; // Addresses 0x00 to 0x3F
        int8_t b_i_raw = (int8_t)eeprom[0x40 + i]; // Addresses 0x40 to 0x7F
        uint8_t delta_alpha_i = eeprom[0x80 + i]; // Addresses 0x80 to 0xBF
        
        int16_t v_ir = (int16_t)ir_frame_raw[i]; //

        // 3a. Calculate V_IR(i,j)_COMPENSATED
        double a_i = ((double)a_common_raw + (double)delta_a_i * pow(2.0, (double)delta_a_scale)) / res_scale; //
        double b_i = (double)b_i_raw / (pow(2.0, (double)b_i_scale) * res_scale); //
        
        double v_ir_offset_comp = (double)v_ir - (a_i + b_i * ta_minus_25); //
        double v_ir_tgc_comp = v_ir_offset_comp - tgc * v_ircp_offset_comp; //
        double v_ir_comp = v_ir_tgc_comp / emissivity; //

        // 3b. Calculate Alpha_comp(i,j)
        double alpha_i = (((double)alpha_0_raw / pow(2.0, (double)alpha_0_scale)) + ((double)delta_alpha_i / pow(2.0, (double)delta_alpha_scale))) / res_scale; //
        double alpha_comp = (1.0 + k_sta * ta_minus_25) * (alpha_i - tgc * alpha_cp); //

        // 3c. Calculate Object Temperature (T_o)
        double sx = k_s4 * pow(pow(alpha_comp, 3.0) * v_ir_comp + pow(alpha_comp, 4.0) * ta_k4, 0.25); // 4th root
        
        double to_k = pow(v_ir_comp / (alpha_comp * (1.0 - k_s4 * 273.15) + sx) + ta_k4, 0.25); // 4th root
        
        // Store the final Celsius value in the output array
        pixel_temp_obj[i] = (float)(to_k - 273.15); //
    }
}
static void Sort_Floats(float* arr, int n) 
{
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j+1]) {
                float temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}

// Helper function to calculate the median of a sorted array
static float Get_Median(float* arr, int n) 
{
    if (n == 0) return 0.0f; // Failsafe
    
    if (n % 2 == 1) {
        return arr[n / 2];
    } else {
        return (arr[(n / 2) - 1] + arr[n / 2]) / 2.0f;
    }
}

/**
  * @brief Groups the 64-pixel array into 8 horizontal zones, computes the median, 
  *        filters out cold anomalies, and returns the final median per zone.
  * @param pixel_temp_obj The input array of 64 calculated To (Object) temperatures.
  * @param horizontal_zones The output array of 8 median temperatures.
  */
void Process_Horizontal_Zones(float *pixel_temp_obj, float *horizontal_zones) 
{
    // Iterate through the 8 horizontal zones
    for (int zone = 0; zone < 8; zone++) 
    {
        float zone_temps[8];
        int count = 0;
        
        // 1. Extract the 8 pixels for this zone
        // Each zone covers 2 columns. 
        // e.g., Zone 0 = Cols 0,1. Zone 1 = Cols 2,3.
        for (int col = zone * 2; col <= (zone * 2) + 1; col++) 
        {
            for (int row = 0; row < 4; row++) 
            {
                // MLX90621 RAM mapping: address = row + (column * 4)
                int index = row + (col * 4);
                zone_temps[count++] = pixel_temp_obj[index];
            }
        }
        
        // 2. Sort the array to find the initial median
        Sort_Floats(zone_temps, 8);
        float initial_median = Get_Median(zone_temps, 8);
        
        // 3. Filter out values less than 15 degrees below the median
        float filtered_temps[8];
        int filtered_count = 0;
        
        for (int i = 0; i < 8; i++) 
        {
            if (zone_temps[i] >= (initial_median - 10.0f)) 
            {
                filtered_temps[filtered_count++] = zone_temps[i];
            }
        }
        
        // 4. Calculate the final median from the filtered values
        // Note: 'filtered_temps' is naturally already sorted because 'zone_temps' was sorted
        horizontal_zones[zone] = Get_Median(filtered_temps, filtered_count);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t i = 0, ret;
  int dev = 0; 
  uint16_t dev_id = 0; 

  uint16_t ptat_raw;
  uint16_t comp_pixel_raw;
  uint16_t ir_frame_raw[64];

  float current_Ta;          // Absolute Chip Temperature
  float pixel_temp_obj[64];  // Final Object Temperatures (To)
  float horizontal_zones[8]; // Array to hold the 8 grouped median values

  MLX_EEPROM_Ta_Coeffs ta_coeffs;
  uint8_t sensor_resolution = 3; // Default 18-bit resolution (11b)
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
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_LPTIM1_Init();
  MX_TIM1_Init();
  MX_USART5_UART_Init();
  
  /* USER CODE BEGIN 2 */
  // 1. I2C Device Scanner
  for(i=1; i<128; i++)
  {
      ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);
      if (ret != HAL_OK) 
      {
          /* No ACK Received At That Address */
      }
      else 
      {
          dev++;
          dev_id = i;
      }
  }

  // 2. Initialize Melexis Sensor (EEPROM Read + Config Write)
  Melexis_Initialize();

  // 3. Extract Ta Coefficients from the EEPROM dump
  ta_coeffs.VTH_L    = eeMLX90621[0xDA];
  ta_coeffs.VTH_H    = eeMLX90621[0xDB];
  ta_coeffs.KT1_L    = eeMLX90621[0xDC];
  ta_coeffs.KT1_H    = eeMLX90621[0xDD];
  ta_coeffs.KT2_L    = eeMLX90621[0xDE];
  ta_coeffs.KT2_H    = eeMLX90621[0xDF];
  ta_coeffs.KT_scale = eeMLX90621[0xD2];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read raw PTAT data
    ptat_raw = MLX90621_ReadPTAT();

    // Calculate Absolute Chip Temperature (Ta)
    current_Ta = Calculate_Ta(ptat_raw, &ta_coeffs, sensor_resolution);

    // Read the Compensation Pixel
    comp_pixel_raw = MLX90621_ReadCompensationPixel();

    // Read the 64 pixels of the IR frame
    MLX90621_ReadWholeFrame(ir_frame_raw);

   Calculate_To_All_Pixels(ir_frame_raw, current_Ta, comp_pixel_raw, eeMLX90621, sensor_resolution, pixel_temp_obj);


  // Group into 8 horizontal zones, apply median filter
    Process_Horizontal_Zones(pixel_temp_obj, horizontal_zones);


    HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
