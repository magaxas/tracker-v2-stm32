/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct  {
 char EET_Time[9];
 char date[11];
 char NS, EW;
 char latitude[12], longitude[12];
} DataPacket;

typedef enum {
    NO_PRESS,
    SINGLE_PRESS,
    LONG_PRESS
} ButtonEvent;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEVICE_ID 1
#define EVENT_ID 1
#define AUTHORIZATION_TOKEN "KPOm4KIpIQHwh0Q7xdFo9Hh0ijO6nKnb"
#define DATA_SEND_PERIOD 120 // max 120 seconds

#define CMD_MAX_LEN 200
#define DATA_BUF_MAX_LEN 5500

#define UART_TIMEOUT 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t buf[BUF_MAX_LEN] = {0};
static char url[50] = "http://vps1953.redfoxcloud.com/api/dataPackets";
static char cmd[CMD_MAX_LEN] = {0};

static uint8_t send_ready = 0;
static uint8_t send_done = 1;
static uint8_t gnss_data_ready = 0;

static uint8_t satNum = 0;
static uint8_t dp_idx = 0; // data packets index
static DataPacket data_packets[DATA_SEND_PERIOD];
static char data[DATA_BUF_MAX_LEN] = {0};

// Push button variables
static const uint32_t DEBOUNCE_MILLIS = 20;
static const uint32_t LONG_MILLIS_MIN = 3000;
static const uint32_t LONG_MILLIS_MAX = 10000;

static uint32_t last_up_ts = 0;
static uint32_t last_down_ts = 0;
static ButtonEvent btn_event = NO_PRESS;

// STOP mode flags
static bool enter_stop_mode = false;
static bool exit_stop_mode = false;

//SD Card driver variables
FATFS FatFs; // Fatfs handle
FIL fil; // File handle
FRESULT fres; // Result after operations
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if (GPIO_Pin == PUSH_BTN_Pin) {
   if (enter_stop_mode) {
     SystemClock_Config();
     HAL_ResumeTick();
     enter_stop_mode = false;
     exit_stop_mode  = true;
     return;
   }

   uint32_t now = HAL_GetTick();
   if (HAL_GPIO_ReadPin(PUSH_BTN_GPIO_Port, PUSH_BTN_Pin) == GPIO_PIN_RESET) {
     last_down_ts = now;
   }
   else {
     last_up_ts = now;

     uint32_t diff = last_up_ts - last_down_ts;
     if (diff > DEBOUNCE_MILLIS) { // Handle debounce

       if (diff < LONG_MILLIS_MIN) {
         btn_event = SINGLE_PRESS;
       }
       else if (diff > LONG_MILLIS_MIN && diff < LONG_MILLIS_MAX) {
         btn_event = LONG_PRESS;
       }
       else {
         btn_event = NO_PRESS;
       }
     }
   }
 }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  if (htim == &htim6 && send_done == 1 && gnss_data_ready == 0) {
    // Repeating every 1s
    gnss_data_ready = 1;
  }
}

void appendDataBuf(uint8_t idx, bool append_last)
{
  strcat((char*)data, (char*)buf);

  if (idx == dp_idx && append_last) { // if last item
    strcat((char*)data, "&");
  } else if (idx != dp_idx) {
    strcat((char*)data, ";");
  }
}

void init()
{
  //Init accelerometer
  LIS2DW12_Init();

  // Mount SD Card
  fres = f_mount(&FatFs, "", 1); // 1=mount now
  if (fres != FR_OK) { // No SD Card found
    BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 3, 200);
  }

  // Set HTTP URL
  sprintf(cmd, "AT+QHTTPURL=%d\r\n", strlen(url));
  bool set_url_ok = send_at_connect_cmd(cmd, url, NULL, NULL, 5);
  // Configure GNSS Constellations (GPS + Galileo)
  bool gnss_config_ok = send_at_cmd("AT+QGPSCFG=\"gnssconfig\",3\r\n");

  //Setup GNSS XTRA
  bool en_xtra_ok = send_at_cmd("AT+QGPSXTRA=1\r\n");
  bool en_xtra_auto_dwnld_ok = send_at_cmd("AT+QGPSCFG=\"xtra_autodownload\",1\r\n");

  // Turn on GNSS
  bool gnss_on_ok = send_at_cmd("AT+QGPS=1\r\n");

  if (en_xtra_ok && en_xtra_auto_dwnld_ok && set_url_ok && gnss_config_ok && gnss_on_ok) {
    BLINK_LED(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 3, 100);
  }
  else {
    BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 3, 100);
  }
}

void add_packet_to_sd_card()
{
  float accelerometerData[3];
  LIS2DW12_ReadXYZ(accelerometerData);
  float vbat = getBatteryVoltage();

  sprintf(
      (char *)buf,
      "%s %s,%.6f,%.6f,%.3f,%.2f,%.2f,%.2f,%d\n",
      data_packets[dp_idx].date,
      data_packets[dp_idx].EET_Time,
      convertToDecimalDegrees(data_packets[dp_idx].latitude, data_packets[dp_idx].NS),
      convertToDecimalDegrees(data_packets[dp_idx].longitude, data_packets[dp_idx].EW),
      vbat,
      accelerometerData[0],
      accelerometerData[1],
      accelerometerData[2],
      satNum
  );

  //Open the file
  fres = f_open(&fil, "packets.txt", FA_OPEN_APPEND | FA_WRITE);
  if (fres != FR_OK) { //File creation/open Error
    BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 1, 200);
    return;
  }
  else {
    // write backup to sd card
    f_puts((char*)buf, &fil);
    f_close(&fil);
  }
}

void send_data()
{
  send_done = 0;
  send_ready = 0;
  send_at_cmd("AT+QGPSCFG=\"priority\",1\r\n"); // Set WWAN priority
  HAL_Delay(150);

  // Fill data
  sprintf(
      (char *)data,
      "authorizationToken=%s&"
      "eventId=%d&"
      "deviceId=%d&"
      "dataPacketsAmount=%d&",
      AUTHORIZATION_TOKEN,
      EVENT_ID,
      DEVICE_ID,
      dp_idx+1
  );

  strcat(data, "latitudes=");
  for (uint8_t i = 0; i < dp_idx+1; i++) {
    sprintf((char *)buf, "%.6f", convertToDecimalDegrees(data_packets[i].latitude, data_packets[i].NS));
    appendDataBuf(i, true);
  }

  strcat(data, "longitudes=");
  for (uint8_t i = 0; i < dp_idx+1; i++) {
    sprintf((char *)buf, "%.6f", convertToDecimalDegrees(data_packets[i].longitude, data_packets[i].EW));
    appendDataBuf(i, true);
  }

  strcat(data, "dates=");
  for (uint8_t i = 0; i < dp_idx+1; i++) {
    sprintf((char *)buf, "%s %s", data_packets[i].date, data_packets[i].EET_Time);
    appendDataBuf(i, false);
  }

  // Send HTTP POST
  sprintf(cmd, "AT+QHTTPPOST=%d\r\n", strlen(data));
  if (send_at_connect_cmd(cmd, data, "+QHTTPPOST:", "+QHTTPPOST: 0,200", 1)) {
    BLINK_LED(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 2, 100);
  }
  else {
    BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 2, 100);
  }

  send_at_cmd("AT+QGPSCFG=\"priority\",0\r\n"); // Set GNSS priority
  dp_idx = 0;
  send_done = 1;
}

uint8_t parse_gnss_data()
{
  uint8_t err_code = 5; // unknown

  if (strstrn((char *)buf, "ERROR", BUF_MAX_LEN) != NULL) {
    err_code = 1; // waiting for position fix
  }
  else if (strstrn((char *)buf, "QGPSLOC", BUF_MAX_LEN) != NULL) {
    err_code = 0;
    // Start parsing GNSS data
    uint16_t skip = 0;
    while (buf[skip] != ' ') {
      if (skip+1 >= BUF_MAX_LEN) break;
      skip++;
    }

    if (skip+10 != BUF_MAX_LEN) {
      char *token = NULL;
      char *delimiter = "\r\n";
      token = strtok((char*)buf+skip+1, delimiter);
      if (token != 0) {
        char *token2;
        token2 = strtok(token, ","); //First token is UTC time
        for (uint8_t i = 1; i < 14; i++) { //last item we need has 13th index
          switch(i) {
          case 1: /* UTC time */
            convertUTCtoEET(token2, data_packets[dp_idx].EET_Time);
            break;
          case 2: /* Raw latitude */
            strcpy(data_packets[dp_idx].latitude, token2);
            break;
          case 3: /* N/S */
            data_packets[dp_idx].NS = (char) token2[0];
            break;
          case 4: /* Raw longtitude */
            strcpy(data_packets[dp_idx].longitude, token2);
            break;
          case 5: /* E/W */
            data_packets[dp_idx].EW = (char) token2[0];
            break;
          case 12: /* date */
            formatDate(token2, data_packets[dp_idx].date);
            break;
          case 13: /* Number of satellites */
            satNum = (uint8_t) strtol(token2, NULL, 10);
            break;
          }
          token2 = strtok(NULL, ",");
        }

        if (strcmp(data_packets[dp_idx].date, "0000-00-00") == 0 ||
            strcmp(data_packets[dp_idx].EET_Time, "00:00:00") == 0 ||
            valid_number(data_packets[dp_idx].latitude) == 0 ||
            valid_number(data_packets[dp_idx].longitude) == 0 ||
            convertToDecimalDegrees(data_packets[dp_idx].latitude, data_packets[dp_idx].NS) == 0.f ||
            convertToDecimalDegrees(data_packets[dp_idx].longitude, data_packets[dp_idx].EW) == 0.f ||
            satNum == 0) {
          err_code = 2; // invalid data packet
        }
      }
      else {
        err_code = 3; // parsing error: strtok failed by \r\n
      }
    }
    else {
      err_code = 4; // parsing error: didn't find space char in buffer
    }
  }

  return err_code;
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  RED_LED_OFF();
  GREEN_LED_OFF();
  BLUE_LED_OFF();
  module_power_up();
  init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (exit_stop_mode) {
      module_power_up();
      init();
      exit_stop_mode = false;
    }
    switch(btn_event) {
      case NO_PRESS:
        break;
      case SINGLE_PRESS:
        float vbat = getBatteryVoltage();
        if (vbat > 3.85f) BLINK_LED(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 3, 100);
        else if (vbat > 3.55f) BLINK_LED(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 2, 100);
        else if (vbat > 3.2f) BLINK_LED(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1, 100);
        else BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 1, 100);
        btn_event = NO_PRESS;
        break;
      case LONG_PRESS:
        BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 1, 1000);
        if (dp_idx > 0) {
          --dp_idx;
          send_data();
        }
        module_power_down();
        f_mount(NULL, "", 0); // de-mount SD card
        HAL_SuspendTick();
        enter_stop_mode = true;
        btn_event = NO_PRESS;
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        break;
    }

    if (gnss_data_ready == 1) {
      memset(buf, 0, sizeof(buf));
      sprintf(cmd, "AT+QGPSLOC=1\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
      HAL_UART_Receive_DMA(&huart1, buf, BUF_MAX_LEN);
      HAL_Delay(100);

      uint8_t err_code = parse_gnss_data();
      if (err_code == 0) {
        add_packet_to_sd_card();

        if (dp_idx+1 >= DATA_SEND_PERIOD) {
          send_ready = 1;
        }
        else {
          ++dp_idx;
        }
      }

      if (err_code == 0 || err_code == 1) { // if success or waiting for position fix
        gnss_data_ready = 0;
      }

      if (err_code > 0) {
        BLINK_LED(RED_LED_GPIO_Port, RED_LED_Pin, 1, 100);
      }
    }

    if (send_ready == 1 && send_done == 1) {
      send_data();
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 100;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 20970;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BLUE_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PWRKEY_Pin|GREEN_LED_Pin|GNNS_LNA_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_CS_Pin|MAIN_DTR_Pin|FPWM_EN_Pin|BUCK_BOOST_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BLUE_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSH_BTN_Pin */
  GPIO_InitStruct.Pin = PUSH_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSH_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWRKEY_Pin GREEN_LED_Pin GNNS_LNA_EN_Pin */
  GPIO_InitStruct.Pin = PWRKEY_Pin|GREEN_LED_Pin|GNNS_LNA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin MAIN_DTR_Pin FPWM_EN_Pin BUCK_BOOST_EN_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|MAIN_DTR_Pin|FPWM_EN_Pin|BUCK_BOOST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL_INT1_Pin */
  GPIO_InitStruct.Pin = ACCEL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACCEL_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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

#ifdef  USE_FULL_ASSERT
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
