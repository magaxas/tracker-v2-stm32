#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#define ADC_TIMEOUT 100

// Accelerometer define
#define LIS2DW12_ADDRESS 0x33
#define LIS2DW12_CTRL1 0x20
#define LIS2DW12_CTRL2 0x21
#define LIS2DW12_CTRL6 0x25
#define LIS2DW12_OUT_X_L 0x28

extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern uint8_t buf[BUF_MAX_LEN];

// LIS2DW12 accelerometer
void LIS2DW12_Init();
void LIS2DW12_ReadXYZ(float *data);

// BG600L-M3 module
void module_power_up();
void module_power_down();
void module_turn_on();
void module_turn_off();
/* Tries to execute cmd until gets OK answer */
bool send_at_cmd(char *cmd);
/* Tries to execute cmd until gets CONNECT and
 * then sends data and retries cmd if
 * doesn't get OK answer */
bool send_at_connect_cmd(char *cmd, char *data, char *key, char *keyval, uint8_t max_cmd_retries);

// LED Controls
void RED_LED_ON();
void RED_LED_OFF();
void GREEN_LED_ON();
void GREEN_LED_OFF();
void BLUE_LED_ON();
void BLUE_LED_OFF();
void BLINK_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t times, uint32_t delay);

// Other functions
int valid_number(char *str);
void convertUTCtoEET(char *utcStr, char *eetStr);
void formatDate(char *date, char *formattedDate);
float convertToDecimalDegrees(const char *latLon, char direction);
char *strstrn(const char *mainStr, const char *subStr, size_t mainStrSize);

void buck_boost_enable();
void buck_boost_disable();

HAL_StatusTypeDef I2C_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void I2C_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);

float getBatteryVoltage();

#endif /* INC_UTILS_H_ */
