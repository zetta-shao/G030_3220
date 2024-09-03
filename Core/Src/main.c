#include <stdio.h>
#include "main.h"
#include "gpiodef.h"
#include "swi2c.h"
#include "swspi.h"
#include "ssd1306.h"
#include "lcd1602sw.h"
#include "INA3221.h"
#include "IP2365.h"
#include "sw35xx_s.h"
#include "ath20.h"
#include "st7920.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
int32_t devices = 0;
int32_t vref;
int8_t env_key1 = 0;
int8_t env_key2 = 0;
int8_t env_key3 = 0;
int8_t _val_ = 0;
swi2c_t si2c1={0}, si2c2={0};
swspi_t sspi1={0};
swspi_t sspi2={0};
//swspi_t sspi3={0};
INA3221_t ina3221 = { 0 };
st7920_t lcd128 = { 0 };
st7920_t lcd129 = { 0 };
stm32_gpio_t lcd_rs = { GPIOA, GPIO_PIN_6 }; //13
stm32_gpio_t lcd_rs2 = { GPIOA, GPIO_PIN_0 }; //7
stm32_gpio_t lcd_mosi = { GPIOA, GPIO_PIN_1 }; //8
stm32_gpio_t lcd_clk = { GPIOA, GPIO_PIN_2 }; //9

#define deffont Font_5x8

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */
void update_ina3221(INA3221_t *ina, SSD1306_t *led) {
	  char str[64];
	  int32_t wV,wI;

	  if(ina->pDev == NULL) {
		  ssd1306_SetCursor(led, 1, 16);
		  ssd1306_WriteString(led, "no INA3221 delect", deffont, 1);
		  return;
	  }

	  //wV = ina3221_getVol_mV(ina, 1); //mV
	  wV = ina3221_getAvgVol(ina, 1);
	  //wI = ina3221_getCur_mA(ina, 1);
	  wI = ina3221_getAvgCur(ina, 1);
	  sprintf(str, "1:%4ldmV %4ldmA    ", wV, wI);
	  //ssd1306_SetCursor(led, 1, 16);
	  //ssd1306_WriteString(led, str, deffont, 1);

	  wV = ina3221_getAvgVol(ina, 2);
	  wI = ina3221_getAvgCur(ina, 2);
	  sprintf(str, "2:%4ldmV %4ldmA    ", wV, wI);
	  //ssd1306_SetCursor(led, 1, 24);
	  //ssd1306_WriteString(led, str, deffont, 1);

	  wV = ina3221_getAvgVol(ina, 3);
	  wI = ina3221_getAvgCur(ina, 3);
	  sprintf(str, "3:%4ldmV %4ldmA    ", wV, wI);
	  //ssd1306_SetCursor(led, 1, 32);
	  //ssd1306_WriteString(led, str, deffont, 1);

	  //ssd1306_UpdateScreen(led);
}
#define ADV_VREF_PREAMP (4096 * 1210)

void init_adc(ADC_HandleTypeDef *d, SSD1306_t *led) {
	uint32_t idx, res = 0;
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

	HAL_ADC_Start(d);
	for(idx=0; idx<4; idx++) {
		HAL_ADC_PollForConversion(d, 1); //wsit for 1mS
		res += HAL_ADC_GetValue(d);
	}
	HAL_ADC_Stop(d);
	res >>= 2;

	vref = ADV_VREF_PREAMP / res;

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

void update_adc(ADC_HandleTypeDef *d, SSD1306_t *led) {
	char str[64];
	uint32_t res, idx;
	//ADC_ChannelConfTypeDef sConfig = {0};
	init_adc(d, led);
	//sConfig.Channel = ADC_CHANNEL_VREFINT;
	//sConfig.Rank = ADC_REGULAR_RANK_1;
	//sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	//HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	//HAL_ADC_Start(d);
	//for(res=0,idx=0; idx<4; idx++) {
	//	HAL_ADC_PollForConversion(d, 1); //wsit for 1mS
	//	res += HAL_ADC_GetValue(d);
	//}
	//HAL_ADC_Stop(d);
	//res >>= 2;

	sprintf(str, "chip_vref %ld %d    ", vref, _val_);

	//sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	//HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(d);
	HAL_ADC_PollForConversion(d, 1); //wsit for 1mS
	res = HAL_ADC_GetValue(d);
	HAL_ADC_Stop(d);
	idx = __LL_ADC_CALC_TEMPERATURE(vref, res, LL_ADC_RESOLUTION_12B);

	sprintf(str, "chip_temp %ld %ld    ", idx, res);
}

int main(void) {
	HAL_Init();
	GPIOinit();
	swi2c_HWinit(&si2c1, &hi2c1);
	swi2c_HWinit(&si2c2, &hi2c2);

	//ina3221_begin(&ina3221, &si2c1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(50);

	swspi_SWinit(&sspi2, &lcd_clk, &lcd_mosi, NULL);
	swspi_setcpol(&sspi2, 1);
	swspi_setcpha(&sspi2, 1);

	//swspi_setbits(&sspi2, 8);
	st7920_init(&lcd129, &sspi2, &lcd_rs2, NULL);
#if 1
  {
	char str[64]; //, vtg[8], cit[8];
	  //float fvalv, fvali;
	//uint16_t wV = ina3221_getDieID(&ina3221);
	//uint16_t wI = ina3221_getManufID(&ina3221);
	//sprintf(str, "INA:%04x:%04x", wV, wI);
	sprintf(str, "INA:%04x:%04x", 0, 0);
	//st7920_string(&lcd129, 0, 0, str);
	st7920_string(&lcd129, 1, 0, "--1--");
	swspi_HWinit(&sspi1, &hspi1);
	swspi_setcpol(&sspi1, 1);
	swspi_setcpha(&sspi1, 1);

	HAL_Delay(1000);
	st7920_string(&lcd129, 2, 0, "--2--");
	st7920_init(&lcd128, &sspi1, &lcd_rs, NULL);

	HAL_Delay(1000);
	st7920_string(&lcd129, 3, 0, "--3--");
	st7920_string(&lcd128, 0, 0, str);

  }
#endif
  _val_ = 0;
  while (1) {
	  //update_ina3221(&ina3221, &SD13061);
	  //update_sw3518(&SW35184, &SD13061);
	  if(env_key1 > 0) { _val_++; env_key1--; }
	  if(env_key2 > 0) { _val_--; env_key2--; }
	  if(env_key3 > 0) { _val_=0; env_key3--; }
	  //update_ath20(&ath20, &SD13061);
	  //HAL_GPIO_WritePin(EVB_LED_P, EVB_LED, GPIO_PIN_SET);
	  HAL_Delay(500);
	  //update_ina3221(&ina3221, &SD13061);
	  //update_sw3518(&SW35184, &SD13061);
	  if(env_key1 > 0) { _val_++; env_key1--; }
	  if(env_key2 > 0) { _val_--; env_key2--; }
	  if(env_key3 > 0) { _val_=0; env_key3--; }
	  //update_ath20(&ath20, &SD13061);
	  //HAL_GPIO_WritePin(EVB_LED_P, EVB_LED, GPIO_PIN_RESET);
	  HAL_Delay(500);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if(htim == &htim14) {
                HAL_GPIO_TogglePin(EVB_LED_P, EVB_LED);
                //HAL_GPIO_TogglePin(LCD_RSP, LCD_RS);
		//st7920_init(&lcd128, &sspi1, &lcd_rs, NULL);
		//st7920_string(&lcd128, 0, 1, "st7920 test");
        }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
	case GPIO_PIN_13: env_key1=1; break;
	case GPIO_PIN_14: env_key2=1; break;
	case GPIO_PIN_15: env_key3=1; break;
	}
}
