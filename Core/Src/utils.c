#include "utils.h"


void LIS2DW12_Init()
{
  I2C_Write(LIS2DW12_ADDRESS, LIS2DW12_CTRL1, 0x55);
  I2C_Write(LIS2DW12_ADDRESS, LIS2DW12_CTRL2, 0x04);
  I2C_Write(LIS2DW12_ADDRESS, LIS2DW12_CTRL6, 0x10);
}

void LIS2DW12_ReadXYZ(float *data)
{
  int16_t rawData[3];
  uint8_t buffer[6];

  /* Read 6 consecutive values from sensor: x,y,z (each is 2 bytes long) */
  I2C_ReadMultiple(LIS2DW12_ADDRESS, LIS2DW12_OUT_X_L, buffer, 6);

  uint8_t i = 0;
  for (i = 0; i < 3; i++) {
    rawData[i] = (((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i];
  }

  /* Convert according to sensitivity */
  for (i = 0; i < 3; i++) {
    data[i] = (float) ((rawData[i]) / 8393.4426f);
  }
}

void module_power_up()
{
  BLINK_LED(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1, 1000);
  HAL_Delay(500);
  buck_boost_enable();
  HAL_Delay(1000);
  module_turn_on();
}

void module_power_down()
{
  module_turn_off();
  buck_boost_disable();
}

void module_turn_on()
{
  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_RESET);
  HAL_Delay(10000); //wait for boot
}

void module_turn_off()
{
  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_RESET);
  HAL_Delay(2000); //wait for shutdown
}

bool send_at_cmd(char *cmd)
{
  uint16_t tries = 0;

  while (1) {
    memset(buf, 0, sizeof(buf));
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
    HAL_UART_Receive_DMA(&huart1, buf, BUF_MAX_LEN);
    HAL_Delay(150);
    if (strstrn((char *)buf, "OK", BUF_MAX_LEN) != NULL) {
      return true;
    }

    if (++tries >= 5) break;
    HAL_Delay(1000);
  }

  return false;
}

bool send_at_connect_cmd(char *cmd, char *data, char *key, char *keyval, uint8_t max_cmd_retries)
{
  uint8_t cmd_retries = 0;
  uint8_t ok_rx_retries = 0;
  uint8_t connect_rx_retries = 0;

  while (1) {
    memset(buf, 0, sizeof(buf));
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
    while (strstrn((char *)buf, "CONNECT", BUF_MAX_LEN) == NULL) {
      HAL_UART_Receive_DMA(&huart1, buf, BUF_MAX_LEN);
      HAL_Delay(200);
      if (++ok_rx_retries >= 20) break;
    }

    if (strstrn((char *)buf, "CONNECT", BUF_MAX_LEN) != NULL) {
      memset(buf, 0, sizeof(buf));
      HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 5000);
      while (strstrn((char *)buf, "OK", BUF_MAX_LEN) == NULL) {
        HAL_UART_Receive_DMA(&huart1, buf, BUF_MAX_LEN);
        HAL_Delay(200);
        if (++connect_rx_retries >= 20) break;
      }

      if (strstrn((char *)buf, "OK", BUF_MAX_LEN) != NULL) {
        if (key != NULL && keyval != NULL) {
          uint8_t keyval_retries = 0;
          memset(buf, 0, sizeof(buf));
          while (strstrn((char *)buf, key, BUF_MAX_LEN) == NULL) {
            HAL_UART_Receive_DMA(&huart1, buf, BUF_MAX_LEN);
            HAL_Delay(200);
            if (strstrn((char *)buf, keyval, BUF_MAX_LEN) != NULL) {
              return true;
            }
            if (++keyval_retries >= 20) break;
          }
        } else {
          return true;
        }
      }
    }

    if (++cmd_retries >= max_cmd_retries) break;
    HAL_Delay(1000);
  }

  return false;
}


char *strstrn(const char *mainStr, const char *subStr, size_t mainStrSize)
{
  char *s1, *s2;
  size_t i = 0;

  while (i < mainStrSize) { // Iterate over the main string
    if (*mainStr == *subStr) { // search for the first character in substring
      // The first character of substring is matched.
      // Check if all characters of substring.
      s1 = mainStr;
      s2 = subStr;

      while (*s1 && *s2) {
        if (*s1 != *s2) break; // not matched
        s1++;
        s2++;
      }

      if(*s2 == NULL) return mainStr; // we reached end of subStr
    }

    mainStr++; // go to next element
    i++;
  }

  return NULL;
}

void RED_LED_ON()
{
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
}

void RED_LED_OFF()
{
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
}

void GREEN_LED_ON()
{
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
}

void GREEN_LED_OFF()
{
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
}

void BLUE_LED_ON()
{
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
}

void BLUE_LED_OFF()
{
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
}

void BLINK_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t times, uint32_t delay)
{
  for (uint8_t i = 0; i < times; i++) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(delay);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    if (i != times-1) HAL_Delay(delay);
  }
}

void convertUTCtoEET(char *utcStr, char *eetStr)
{
  if (strlen(utcStr) != 10) { // If input string is not in the expected format
    strcpy(eetStr, "00:00:00");
    return;
  }

  int hh, mm, ss;
  if (sscanf(utcStr, "%2d%2d%2d", &hh, &mm, &ss) != 3) { // Parse the UTC time
    strcpy(eetStr, "00:00:00");
    return;
  }

  hh = (hh + 3) % 24; // Add 3 hours for the EET timezone
  sprintf(eetStr, "%02d:%02d:%02d", hh, mm, ss); // Format the EET time
}

void formatDate(char *date, char *formattedDate)
{
  if (strlen(date) != 6) {
    strcpy(formattedDate, "0000-00-00");
    return;
  }

  int y, m, d;
  if (sscanf(date, "%2d%2d%2d", &d, &m, &y) != 3) {
    strcpy(formattedDate, "0000-00-00");
    return;
  }

  y += 2000;
  sprintf(formattedDate, "%04d-%02d-%02d", y, m, d);
}

int valid_number(char *str)
{
    int i = 0, j = strlen(str) - 1;

    // Handling whitespaces
    while (i < strlen(str) && str[i] == ' ')
        i++;
    while (j >= 0 && str[j] == ' ')
        j--;

    if (i > j)
        return 0;

    // if string is of length 1 and the only
    // character is not a digit
    if (i == j && !(str[i] >= '0' && str[i] <= '9'))
        return 0;

    // If the 1st char is not '+', '-', '.' or digit
    if (str[i] != '.' && str[i] != '+'
        && str[i] != '-' && !(str[i] >= '0' && str[i] <= '9'))
        return 0;

    // To check if a '.' or 'e' is found in given
    // string. We use this flag to make sure that
    // either of them appear only once.
    bool flagDotOrE = false;

    for (i; i <= j; i++) {
        // If any of the char does not belong to
        // {digit, +, -, ., e}
        if (str[i] != 'e' && str[i] != '.'
            && str[i] != '+' && str[i] != '-'
            && !(str[i] >= '0' && str[i] <= '9'))
            return 0;

        if (str[i] == '.') {
            // checks if the char 'e' has already
            // occurred before '.' If yes, return 0.
            if (flagDotOrE == true)
                return 0;

            // If '.' is the last character.
            if (i + 1 > strlen(str))
                return 0;

            // if '.' is not followed by a digit.
            if (!(str[i + 1] >= '0' && str[i + 1] <= '9'))
                return 0;
        }

        else if (str[i] == 'e') {
            // set flagDotOrE = 1 when e is encountered.
            flagDotOrE = true;

            // if there is no digit before 'e'.
            if (!(str[i - 1] >= '0' && str[i - 1] <= '9'))
                return 0;

            // If 'e' is the last Character
            if (i + 1 > strlen(str))
                return 0;

            // if e is not followed either by
            // '+', '-' or a digit
            if (str[i + 1] != '+' && str[i + 1] != '-'
                && (str[i + 1] >= '0' && str[i] <= '9'))
                return 0;
        }
    }

    /* If the string skips all above cases, then
    it is numeric*/
    return 1;
}

float convertToDecimalDegrees(const char *latLon, char direction)
{
  char deg[4] = {0};
  char *dot, *min;
  int len;
  float dec = 0;

  if ((dot = strchr(latLon, '.'))) { /* if decimal point was found */
    min = dot - 2; /* mark the start of minutes 2 chars back */
    len = min - latLon; /* find the length of degrees */
    strncpy(deg, latLon, len); /* copy the degree string to allow conversion to float */

    dec = atof(deg) + atof(min) / 60; /* convert to float */
    if (direction == 'S' || direction == 'W') dec *= -1;
  }

  return dec;
}

void buck_boost_enable()
{
  HAL_GPIO_WritePin(FPWM_EN_GPIO_Port, FPWM_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUCK_BOOST_EN_GPIO_Port, BUCK_BOOST_EN_Pin, GPIO_PIN_SET);
}

void buck_boost_disable()
{
  HAL_GPIO_WritePin(FPWM_EN_GPIO_Port, FPWM_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUCK_BOOST_EN_GPIO_Port, BUCK_BOOST_EN_Pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef I2C_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  return HAL_I2C_Mem_Read(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length, 1000);
}

void I2C_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Value, 1, 1000);
}

float getBatteryVoltage()
{
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT);
  uint32_t raw = HAL_ADC_GetValue(&hadc);
  return (float)raw/4095.f * 3.3f * 1.28f;
}
